import serial
import time
import threading


class Piezo:
    def __init__(self, port):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            rtscts=False,
            dsrdtr=False
        )
        self.keep_running = True
        self.lock = threading.Lock()
        self.buffer = b""
        self.buffer = bytearray()
        # self.stop_event = threading.Event()

    # 计算数据长度
    def calculate_length(self, input_data):
        length = len(input_data)
        return length

    # CRC校验
    def calculate_crc(self, data):
        crc = 0xFFFFFFFF
        polynomial = 0x04C11DB7
        crc_data = bytes(data)

        for byte in crc_data:
            crc ^= byte << 24
            for _ in range(8):
                if crc & 0x80000000:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
        crc_value = crc & 0xFFFFFFFF

        return [
            crc_value & 0xFF,
            (crc_value >> 8) & 0xFF,
            (crc_value >> 16) & 0xFF,
            (crc_value >> 24) & 0xFF
        ]

    # 生成指令
    def command(self, head, input_data):
        length = self.calculate_length(input_data)
        crc = self.calculate_crc([head] + [length] + input_data)
        comm = [head, length] + input_data + crc
        return comm

    # 发送指令
    def send(self, head, input_data):
        try:
            comm = self.command(head, input_data)
            self.ser.write(bytes(comm))
            # print(f"发送指令: {bytes(comm).hex()}")
        except Exception as e:
            print(f"Error writing to device: {e}")

    # 读取指令
    def read(self, frame_length):
        try:
            frame = self.ser.read(frame_length)
            # print(f"收到指令: {frame.hex()}")
            return frame
        except Exception as e:
            print(f"Error reading from device: {e}")
            return None

    # 数据拆分
    def parse_response(self, response):
        if response and len(response) > 2:
            head = response[0]
            length = response[1]
            data = response[2:2 + length]
            crc = response[2 + length:2 + length + 4]
            return head, length, data, crc
        return None, None, None, None

    # 错误解析
    def parse_status(self, status_byte):
        status_dict = {
            0x00: "通讯正常",
            0x01: "命令出错",
            0x02: "数据长度出错",
            0x03: "数据内容出错",
            0x04: "数据内容越界",
            0x05: "系统执行错误",
            0x06: "CRC校验出错",
            0x10: "通信连接未建立",
            0x11: "手柄未连接",
            0x12: "系统未进入开始状态",
            0x13: "自检未成功",
            0x14: "自检进行中",
            0x15: "超声输出中",
            0x16: "液流单独输出中",
            0x17: "系统进入不可回复状态",
            0x20: "系统授权失败"
        }

        if status_byte in status_dict:
            return status_dict[status_byte]
        else:
            return "未知状态码"

    def process_data(self, raw_data, data_format):
        # 数据包的标志（包含三个字节）
        header = bytes(data_format[:2])
        data_length = data_format[1]

        # 查找数据包的起始位置
        start_index = raw_data.find(header)
        if start_index == -1:
            # 未找到数据包标志
            return None

        # 计算数据包总长度
        packet_length = 2 + data_length + 4

        # 确认缓冲区中有足够的数据
        if len(raw_data) < start_index + packet_length:
            # 数据不足，无法获取完整数据包
            return None

        # 提取一个完整的数据包
        data_packet = raw_data[start_index:start_index + packet_length]
        # 解析数据包
        data = data_packet[2:2 + data_length]  # data部分
        crc_received = data_packet[2 + data_length:6 + data_length]

        # 计算并验证CRC
        crc_calculated = self.calculate_crc(data_packet[:2 + data_length])
        if bytes(crc_calculated) == crc_received:
            # 如果CRC匹配，形成完整数据
            return data
        else:
            # CRC不匹配，丢弃数据
            return None

    # 连接设备
    def connect(self):
        head = 0x55
        input_data = [0x50, 0x01, 0x73, 0x7b, 0x00, 0x00, 0xff, 0xff, 0x84, 0x8c]
        self.send(head, input_data)
        time.sleep(0.1)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x50]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("连接成功")
            return True
        else:
            print(f"连接失败: {status_message}")
            return False

    # 心跳检测
    def heartbeat(self):
        head = 0x55
        input_data = [0x51]
        self.send(head, input_data)
        time.sleep(0.05)
        self.read(8)

    # 心跳检测线程
    def start_heartbeat(self):
        def run():
            # while not self.stop_event.is_set():
            while self.keep_running:
                self.heartbeat()
                time.sleep(3)

        threading.Thread(target=run, daemon=True).start()

    # 系统复位
    def reset(self):
        head = 0x55
        input_data = [0x52]
        self.send(head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x52]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("复位成功")
            return True
        else:
            print(f"复位失败: {status_message}")
            return False

    # 开始工作
    def start(self):
        head = 0x55
        input_data = [0x53, 0x01]
        self.send(head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x53]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("开始")
            return True
        else:
            print(f"开始失败: {status_message}")
            return False

    # 停止工作
    def stop(self):
        head = 0x55
        input_data = [0x53, 0x00]
        self.send(head, input_data)
        # time.sleep(0.1)
        # raw_data = self.read(10)
        # data_format = [0x55, 0x02, 0x53]
        # data = self.process_data(raw_data, data_format)
        # status_byte = data[1]
        # status_message = self.parse_status(status_byte)
        # if status_byte == 0x00:
        #     print("停止")
        #     return True
        # else:
        #     print(f"停止失败: {status_message}")
        #     return False

    # 自检
    def selftest(self):
        head = 0x55
        input_data = [0x54]
        self.send(head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x54]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("自检成功")
            return True
        else:
            print(f"自检失败: {status_message}")
            return False

    # 功率设置 1-5档
    def set_power(self, num):
        head = 0x55
        input_data = [0x55, num]
        self.send(head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x55]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("功率设置成功")
            return True
        else:
            print(f"功率设置失败: {status_message}")
            return False

    # 流量设置 1-10档
    def set_flow(self, num):
        Head = 0x55
        input_data = [0x56, num]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x56]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("流量设置成功")
            return True
        else:
            print(f"流量设置失败: {status_message}")
            return False

    # 脉冲设置 1-6档
    def set_pulse(self, num):
        Head = 0x55
        input_data = [0x57, num]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x57]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("脉冲设置成功")
            return True
        else:
            print(f"脉冲设置失败: {status_message}")
            return False

    # 启动超声
    def power_on(self):
        Head = 0x55
        input_data = [0x58, 0x01]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x58]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("超声启动")
            return True
        else:
            print(f"超声启动失败: {status_message}")
            return False

    # 关闭超声
    def power_off(self):
        Head = 0x55
        input_data = [0x58, 0x00]
        self.send(Head, input_data)
        # time.sleep(0.1)
        # raw_data = self.read(8)
        # data_format = [0x55, 0x02, 0x58]
        # data = self.process_data(raw_data, data_format)
        # status_byte = data[1]
        # status_message = self.parse_status(status_byte)
        # if status_byte == 0x00:
        #     print("关闭超声")
        #     return True
        # else:
        #     print(f"关闭超声失败: {status_message}")
        #     return False

    # 启动液流
    def flow_on(self):
        Head = 0x55
        input_data = [0x59, 0x01]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x59]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("启动液流")
            return True
        else:
            print(f"启动液流失败: {status_message}")
            return False

    # 关闭液流
    def flow_off(self):
        Head = 0x55
        input_data = [0x59, 0x00]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(8)
        data_format = [0x55, 0x02, 0x59]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            print("关闭液流成功")
            return True
        else:
            print(f"关闭液流失败: {status_message}")
            return False

    # 查询系统状态
    def check_state(self):
        Head = 0x55
        input_data = [0x5A]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(12)
        data_format = [0x55, 0x06, 0x5A]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            handle_status = "已接入" if data[2] == 0x01 else "未接入"
            self_test_status = "已通过" if data[3] == 0x01 else "未通过"
            operational_status_dict = {
                0x00: "停止状态",
                0x01: "开始状态",
                0x02: "超声输出状态",
                0x03: "自检进行状态",
                0x04: "液流单独工作状态"
            }
            operational_status = operational_status_dict.get(data[4], "未知状态")
            alarm_status = "无报警" if data[5] == 0x00 else "有报警"

            print(
                f"状态检测正常: 手柄{handle_status}, 自检{self_test_status}, {operational_status}, {alarm_status}")
            return True
        else:
            print(f"连接失败: {status_message}")
            return False

    # 查询当前工作档位
    def check_level(self):
        Head = 0x55
        input_data = [0x5B]
        self.send(Head, input_data)
        time.sleep(0.05)
        raw_data = self.read(11)
        data_format = [0x55, 0x05, 0x5B]
        data = self.process_data(raw_data, data_format)
        status_byte = data[1]
        status_message = self.parse_status(status_byte)
        if status_byte == 0x00:
            handle_status = data[2]
            self_test_status = data[3]
            operational_status = data[4]

            print(
                f"当前档位: 功率{handle_status}, 流量{self_test_status}, 脉冲{operational_status}")
            return True
        else:
            print(f"连接失败: {status_message}")
            return False

    # 断开连接
    def disconnect(self):
        Head = 0x55
        input_data = [0x50, 0x00, 0x73, 0x7b, 0x00, 0x00, 0xff, 0xff, 0x84, 0x8c]
        self.send(Head, input_data)
        # time.sleep(0.1)
        # raw_data = self.read(8)
        # data_format = [0x55, 0x02, 0x50]
        # data = self.process_data(raw_data, data_format)
        # status_byte = data[1]
        # status_message = self.parse_status(status_byte)
        # if status_byte == 0x00:
        #     print("断开连接")
        #     return True
        # else:
        #     print(f"连接失败: {status_message}")
        #     return False

    # 获取声阻抗数据
    def get_impedance(self):
        raw_data = self.read(17)  # 从串口读取数据
        data_format = [0xaa, 0x03, 0xb0]
        data = self.process_data(raw_data, data_format)
        if data is not None:
            impedance_data = (data[2] << 8) | data[1]
            print(f"阻抗: {impedance_data}")
            return impedance_data

    def start_impedance(self):
        def run():
            # while not self.stop_event.is_set():
            while self.keep_running:
                self.get_impedance()

        threading.Thread(target=run, daemon=True).start()

    # 停止设备
    def cut_down(self):
        self.keep_running = False
        # self.stop_event.set()
        print("准备关闭")
        self.power_off()
        print("关闭超声")
        time.sleep(0.1)
        self.stop()
        print("停止运行")
        time.sleep(0.1)
        self.disconnect()
        print("关闭连接")
        time.sleep(0.1)
        self.ser.close()
        print("关闭串口")


if __name__ == "__main__":
    device = Piezo(port='COM3')

    device.connect()
    device.start_heartbeat()
    time.sleep(0.1)
    device.check_state()
    time.sleep(0.1)
    # # # # device.reset()
    device.start()
    time.sleep(0.1)
    device.set_power(5)
    time.sleep(0.1)
    device.set_flow(4)
    time.sleep(0.1)
    device.set_pulse(6)
    time.sleep(0.1)
    # device.selftest()  # 自检必须在功率、流量脉冲设置之后
    time.sleep(1)
    device.power_on()
    time.sleep(0.1)
    device.start_impedance()
    start_time = time.time()
    while True:
        if time.time() - start_time > 5:
            break
    time.sleep(0.1)
    # device.ser.close()
    print("结束")
    device.cut_down()
    print("正式结束")
