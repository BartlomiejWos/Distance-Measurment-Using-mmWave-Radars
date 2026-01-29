import serial
from .interfaces import UartTransport

class PySerialUart(UartTransport):
    def __init__(self, logger):
        self._port = "COM22"
        self._baudrate = 921600
        self._timeout = 3.0
        self.logger = logger
        self._ser = None

    def connect(self) -> None:
        self._ser = serial.Serial(self._port,
                                  self._baudrate,
                                  timeout=self._timeout)
        self.logger.info("Connected")

    def close(self) -> None:
        if self._ser:
            self._ser.close()
            self._ser = None

    def send(self,cmd):
        self._ser.write((cmd + '\n').encode('utf-8'))
        print(f">>> sended {cmd}\n")
        try:
            for _ in range(3):
                resp = self._ser.readline().decode('utf-8').strip()
                # if len(resp) == 0:
                #     break
                print(f"<<< {resp}")
                if "Done" in resp:
                    print(f"<<< {resp}")
                    break
        except Exception as e:
            print(f"Error: {e}")
            return False
        return True
