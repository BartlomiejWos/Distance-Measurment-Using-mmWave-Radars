from typing import Protocol

class UartTransport(Protocol):
    def connect(self) -> None:
        pass
    def close(self) -> None:
        pass
    def send(self, cmd: str) -> None:
        pass
