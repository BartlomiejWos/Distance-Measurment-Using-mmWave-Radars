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
            self.logger.error("UART connect failed on %s: %s", self.port, exc)
            return False
        self.logger.info("UART connected on %s", self.port)
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
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.logger.debug(">>> %s", cmd)
            for _ in range(3):
                resp = self.ser.readline().decode("utf-8").strip()
                self.logger.debug("<<< %s", resp)
                if "Done" in resp:
                    return True
        except Exception as exc:
            self.logger.error("UART send error: %s", exc)
            return False
        self.logger.error(
            "UART send timeout waiting for 'Done' after: %s", cmd
        )
        return False
