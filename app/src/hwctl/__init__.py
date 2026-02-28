import logging
from src.ports.uart import UartPort
from src.ports.dca1000 import DCA1000Port


class HwCtl:
    def __init__(
        self,
        uart: UartPort,
        radar_api: DCA1000Port,
        logger: logging.Logger,
    ) -> None:
        self.uart = uart
        self.radar_api = radar_api
        self.logger = logger
