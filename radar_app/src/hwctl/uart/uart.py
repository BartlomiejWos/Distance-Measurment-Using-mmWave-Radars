import logging
import serial


class Uart:
    def __init__(self, port: str, logger: logging.Logger):
        self.port = port
        self.baudrate = 921600
        self.timeout = 3.0
        self.logger = logger
        self.ser: serial.Serial | None = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout
            )
        except serial.SerialException as exc:
            self.logger.error(f"UART connect failed on {self.port}: {exc}")
            return False
        return True

    def close(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def send(self, cmd: str) -> bool:
        if self.ser is None:
            self.logger.error("UART send called before connect")
            return False
        try:
            self.ser.write((cmd + "\r\n").encode("utf-8"))
            self.ser.flush()
            self.logger.debug(f">>> {cmd}")
            for _ in range(3):
                resp = self.ser.readline().decode("utf-8").strip()
                self.logger.debug(f"<<< {resp}")
                if "Done" in resp:
                    return True
        except Exception as exc:
            self.logger.error(f"UART send error: {exc}")
            return False
        self.logger.error(
            f"UART send timeout waiting for 'Done' after: {cmd}"
        )
        return False
