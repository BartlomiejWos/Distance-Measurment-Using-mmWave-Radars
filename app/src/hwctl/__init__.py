import logging
from src.ports.uart import Uart
from src.ports.dca1000 import DCA1000


class HwCtl:
    def __init__(
        self,
        uart: Uart,
        radar_api: DCA1000,
        logger: logging.Logger,
    ) -> None:
        self.uart = uart
        self.radar_api = radar_api
        self.logger = logger
