import logging
import ports


class HwCtl:
    def __init__(
        self,
        uart: ports.Uart,
        radar_api: ports.DCA1000,
        logger: logging.Logger,
    ) -> None:
        self.uart = uart
        self.radar_api = radar_api
        self.logger = logger
