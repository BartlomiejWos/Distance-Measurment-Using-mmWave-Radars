import logging
import serial


class Uart:
    def __init__(self, port: str, logger: logging.Logger):
        self._port = port
        self._baudrate = 921600
        self._timeout = 3.0
        self._logger = logger
        self._ser: serial.Serial | None = None

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                self._port, self._baudrate, timeout=self._timeout
            )
        except serial.SerialException as exc:
            self._logger.error("UART connect failed on %s: %s", self._port, exc)
            return False
        self._logger.info("UART connected on %s", self._port)
        return True

    def close(self) -> None:
        if self._ser:
            self._ser.close()
            self._ser = None

    def send(self, cmd: str) -> bool:
        if self._ser is None:
            self._logger.error("UART send called before connect")
            return False
        try:
            self._ser.write((cmd + "\n").encode("utf-8"))
            self._logger.debug(">>> %s", cmd)
            for _ in range(3):
                resp = self._ser.readline().decode("utf-8").strip()
                self._logger.debug("<<< %s", resp)
                if "Done" in resp:
                    return True
        except Exception as exc:
            self._logger.error("UART send error: %s", exc)
            return False
        self._logger.error(
            "UART send timeout waiting for 'Done' after: %s", cmd
        )
        return False
