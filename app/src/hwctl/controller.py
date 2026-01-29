import time

from app.src.ports.uart import UartTransport
from app.src.ports.radar_api import RadarApi


class HwCtl:
    def __init__(self, uart: UartTransport, radar: RadarApi, logger):
        self.uart = uart
        self.radar = radar
        self.log = logger

    def apply_commands(self, commands: list[str]) -> None:
        for cmd in commands:
            self.uart.send(cmd)

    def one_capture(self, file_prefix: str, commands: list[str]) -> None:
        # self.radar.dca1000_config_init()
        # self.radar.read_rfdc_card_fpga_version()

        self.uart.connect()
        try:
            self.apply_commands(commands)
            print("IHAAAAA")
            # self.radar.dca_set_file_prefix(file_prefix)
            # self.log.info("Start of record: %s", file_prefix)

            # self.radar.dca_trigger_record()
            # self.uart.send("sensorStart")

            # time.sleep(20)

            # self.log.info("OK: 1 Frame Recorded")
            # self.radar.dca_stop_record()
        finally:
            self.uart.close()
