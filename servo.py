import serial
import time
import asyncio

# 常量定义
GET_LOW_BYTE = lambda A: A & 0xFF  # 获取低8位
GET_HIGH_BYTE = lambda A: (A >> 8) & 0xFF  # 获取高8位
BYTE_TO_HW = lambda A, B: ((A & 0xFF) << 8) | (B & 0xFF)  # A 是高字节, B 是低字节

ID_ALL = 254  # 广播 ID (所有舵机)

LOBOT_SERVO_FRAME_HEADER = 0x55  # 帧头
LOBOT_SERVO_MOVE_TIME_WRITE = 1  # 舵机按时间移动指令
LOBOT_SERVO_POS_READ = 28  # 读取舵机位置指令
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31  # 写入舵机卸载/加载指令

class LobotServo:
    def __init__(self, port, servo_id, baudrate=115200, timeout=0.1):
        """
        初始化 LobotServo 控制器。
        :param port: 串口名称 (例如，Windows 上是 "COM3"，Linux 上是 "/dev/ttyUSB0")。
        :param servo_id: 舵机ID (0-253，或 ID_ALL 用于广播)。
        :param baudrate: 串口通信波特率 (默认 115200)。
        :param timeout: 串口读取超时时间，单位秒 (默认 0.1)。
        """
        if not (0 <= servo_id <= 253 or servo_id == ID_ALL):
            raise ValueError("舵机ID必须在0到253之间，或者为ID_ALL (254)。")

        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baudrate
        self._serial.timeout = timeout
        self._id = servo_id
        self._pos = 0  # 存储最后一次命令的位置 (不一定是当前实际位置)

    def begin(self):
        """
        打开串口。
        """
        try:
            if not self._serial.is_open:
                self._serial.open()
        except serial.SerialException as e:
            raise serial.SerialException(f"无法打开串口 {self._serial.port}: {e}")

    def close(self):
        """
        关闭串口。
        """
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _calculate_checksum(self, packet_bytes_for_summing):
        """
        计算数据包的校验和。
        """
        temp_sum = 0
        for byte_val in packet_bytes_for_summing:
            temp_sum += byte_val
        return (~temp_sum) & 0xFF  # 取反并取低8位

    def _send_command(self, command_id, params=None):
        """
        构造并向舵机发送命令数据包。
        """
        if not self._serial.is_open:
            raise serial.SerialException("串口未打开。")

        if params is None:
            params = []

        length_field = 3 + len(params)
        packet_part_for_checksum = bytes([self._id, length_field, command_id] + params)
        checksum = self._calculate_checksum(packet_part_for_checksum)

        full_packet = bytes([
            LOBOT_SERVO_FRAME_HEADER,
            LOBOT_SERVO_FRAME_HEADER
        ]) + packet_part_for_checksum + bytes([checksum])

        self._serial.write(full_packet)

    def move(self, position, time_ms=1000):
        """
        在给定时间内将舵机移动到指定位置。
        """
        self.begin()  # 打开串口
        try:
            position = max(0, min(position, 1000))
            time_ms = max(0, min(time_ms, 30000))

            params = [
                GET_LOW_BYTE(position),
                GET_HIGH_BYTE(position),
                GET_LOW_BYTE(time_ms),
                GET_HIGH_BYTE(time_ms)
            ]
            self._send_command(LOBOT_SERVO_MOVE_TIME_WRITE, params)
            self._pos = position
            time.sleep(0.005)
        finally:
            self.close()  # 发送完成后关闭串口

    def read_position(self):
        """
        从舵机读取当前角度位置。
        """
        self.begin()  # 打开串口
        try:
            self._serial.reset_input_buffer()
            self._send_command(LOBOT_SERVO_POS_READ)

            response_payload = self.receive_response()
            if response_payload and len(response_payload) == 3 and response_payload[0] == LOBOT_SERVO_POS_READ:
                pos_l = response_payload[1]
                pos_h = response_payload[2]
                current_pos = BYTE_TO_HW(pos_h, pos_l)
                self._pos = current_pos
                return current_pos
            return None
        finally:
            self.close()  # 读取完成后关闭串口

    def receive_response(self, response_timeout_duration=0.5):
        """
        尝试接收并验证来自舵机的响应数据包。
        """
        start_time = time.time()
        frame_started = False
        header_count = 0
        received_packet_bytes = bytearray()
        expected_length_field_value = 0

        while time.time() - start_time < response_timeout_duration:
            if self._serial.in_waiting > 0:
                byte_read = self._serial.read(1)
                if not byte_read:
                    continue
                byte = byte_read[0]

                if not frame_started:
                    if byte == LOBOT_SERVO_FRAME_HEADER:
                        header_count += 1
                        if header_count == 2:
                            frame_started = True
                            received_packet_bytes.clear()
                            expected_length_field_value = 0
                    else:
                        header_count = 0
                else:
                    received_packet_bytes.append(byte)
                    if expected_length_field_value == 0 and len(received_packet_bytes) >= 2:
                        expected_length_field_value = received_packet_bytes[1]
                        if not (3 <= expected_length_field_value <= 30):
                            frame_started = False
                            header_count = 0
                            continue

                    if expected_length_field_value > 0 and \
                       len(received_packet_bytes) == expected_length_field_value + 1:
                        data_to_checksum = received_packet_bytes[:-1]
                        received_checksum = received_packet_bytes[-1]
                        calculated_checksum = self._calculate_checksum(data_to_checksum)

                        if calculated_checksum == received_checksum:
                            payload = bytes(received_packet_bytes[2:-1])
                            return payload
                        else:
                            frame_started = False
                            header_count = 0
                            continue
            else:
                time.sleep(0.001)

        return None

# 示例：控制单个舵机
if __name__ == "__main__":
    SERIAL_PORT = "COM3"  # 修改为你的串口
    SERVO_ID = 1  # 修改为你的舵机 ID

    servo = LobotServo(SERIAL_PORT, SERVO_ID)
    try:
        print("正在将舵机移动到位置 500...")
        servo.move(500, 1000)

        print("读取舵机当前位置...")
        position = servo.read_position()
        if position is not None:
            print(f"舵机当前位置: {position}")
        else:
            print("读取舵机位置失败。")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        servo.close()
