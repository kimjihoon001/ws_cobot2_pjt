import serial
import serial.tools.list_ports
import time

PORT = "/dev/ttyACM0"  # 자동 감지 실패시 폴백 (ls /dev/tty* 로 실제 포트 확인 후 필요시 수정)
BAUD = 115200
ARDUINO_VID = 0x2341  # Arduino SA 벤더 ID


def find_arduino_port(default=PORT):
    """연결된 포트 중 Arduino 벤더 ID(2341)를 가진 걸 자동으로 찾는다.
    USB 재연결/다른 시리얼 장치 유무에 따라 /dev/ttyACM0, ttyACM1 등으로
    번호가 매번 바뀔 수 있어서, 하드코딩된 포트 대신 이걸 우선 사용."""
    for port in serial.tools.list_ports.comports():
        if port.vid == ARDUINO_VID:
            return port.device
    return default


class ConveyorController:
    def __init__(self, port=None, baud=BAUD):
        port = port or find_arduino_port()
        print(f"시리얼 포트: {port}")
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # 아두이노 자동 리셋 후 부팅 대기

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        print(f"-> {cmd}")

    def run(self):
        self.send("RUN")

    def stop(self):
        self.send("STOP")

    def move(self, steps):
        self.send(f"MOVE:{steps}")
        # 5500 중간쯤
    def set_speed(self, speed):
        self.send(f"SPEED:{speed}")

    def set_accel(self, accel):
        self.send(f"ACCEL:{accel}")

    def set_enable(self, enabled):
        self.send(f"ENA:{1 if enabled else 0}")

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    conveyor = ConveyorController()
    print("명령 입력: RUN / STOP / MOVE:5000 / SPEED:100 / ACCEL:20 / ENA:0 / ENA:1 / exit")
    try:
        while True:
            cmd = input("> ").strip()
            if not cmd:
                continue
            if cmd.lower() == "exit":
                break
            conveyor.send(cmd)
    finally:
        conveyor.close()
