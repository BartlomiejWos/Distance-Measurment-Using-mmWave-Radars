import logging
import serial


class PySerialUart:
    def __init__(self, port: str, logger: logging.Logger):
        self._port = port
        self._baudrate = 921600
        self._timeout = 3.0
        self._logger = logger
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        self._ser = serial.Serial(
            self._port, self._baudrate, timeout=self._timeout
        )
        self._logger.info("UART connected on %s", self._port)

    def close(self) -> None:
        if self._ser:
            self._ser.close()
            self._ser = None

    def send(self, cmd: str) -> bool:
        self._ser.write((cmd + "\n").encode("utf-8"))
        self._logger.debug(">>> %s", cmd)
        try:
            for _ in range(3):
                resp = self._ser.readline().decode("utf-8").strip()
                self._logger.debug("<<< %s", resp)
                if "Done" in resp:
                    return True
        except Exception as exc:
            self._logger.error("UART send error: %s", exc)
            return False
        return True
