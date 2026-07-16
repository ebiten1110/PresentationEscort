import time
import serial
from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT


class SerialSender:
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD, timeout=SERIAL_TIMEOUT):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        print(f"[Serial] Opening {self.port} at {self.baud} bps")

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout,
        )

        # ESP32はシリアル接続時にリセットされることがあるので少し待つ
        time.sleep(2.0)

        print("[Serial] Opened")

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("[Serial] Closed")

    def send_command(self, command):
        if self.ser is None or not self.ser.is_open:
            print("[Serial] ERROR: Serial port is not open")
            return False

        command = command.strip().upper()
        message = command + "\n"

        self.ser.write(message.encode("utf-8"))
        self.ser.flush()

        print(f"[Serial] Sent: {command}")
        return True

    def read_lines(self, duration=1.0):
        """
        ESP32から返ってくるSerial.printlnを少し読む
        """
        if self.ser is None or not self.ser.is_open:
            return

        start_time = time.time()

        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"[ESP32] {line}")


def main():
    sender = SerialSender()

    try:
        sender.open()

        print("[Serial] Test commands start")

        commands = [
            "STATUS",
            "STAND",
            "LIGHT_TOGGLE",
            "HEAD_LEFT",
            "HEAD_RIGHT",
            "HEAD_CENTER",
            "LEFT",
            "RIGHT",
            "STOP",
            "STATUS",
        ]

        for command in commands:
            sender.send_command(command)
            sender.read_lines(duration=0.8)
            time.sleep(1.0)

        print("[Serial] Test commands finished")

    except serial.SerialException as e:
        print("[Serial] ERROR: SerialException")
        print(e)
        print("SERIAL_PORT が合っているか確認してください")
        print("例: /dev/ttyUSB0 または /dev/ttyACM0")

    finally:
        sender.close()


if __name__ == "__main__":
    main()