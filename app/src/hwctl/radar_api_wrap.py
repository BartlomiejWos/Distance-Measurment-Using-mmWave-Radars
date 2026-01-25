import os, sys
from .interfaces import RadarApi

class RadarApiPyd(RadarApi):
    def __init__(self, dll_path: str):
        if dll_path not in sys.path:
            sys.path.append(dll_path)
        os.add_dll_directory(dll_path)
        from . import radar_api as ra
        self._ra = ra

    def dca1000_config_init(self) -> None: self._ra.dca1000_config_init()
    def read_rfdc_card_fpga_version(self) -> None: self._ra.read_rfdc_card_fpga_version()
    def dca_set_file_prefix(self, prefix: str) -> None: self._ra.dca_set_file_prefix(prefix)
    def dca_trigger_record(self) -> None: self._ra.dca_trigger_record()
    def dca_stop_record(self) -> None: self._ra.dca_stop_record()
