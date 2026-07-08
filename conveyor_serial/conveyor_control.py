import serial
import time

PORT = "/dev/ttyACM0"  # ls /dev/tty* 로 실제 포트 확인 후 필요시 수정
BAUD = 115200


class ConveyorController:
    def __init__(self, port=PORT, baud=BAUD):
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
