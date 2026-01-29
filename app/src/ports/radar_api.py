from typing import Protocol

class RadarApi(Protocol):
    def dca1000_config_init(self) -> None:
        pass

    def read_rfdc_card_fpga_version(self) -> None:
        pass

    def dca_set_file_prefix(self, prefix: str) -> None:
        pass

    def dca_trigger_record(self) -> None:
        pass

    def dca_stop_record(self) -> None:
        pass
