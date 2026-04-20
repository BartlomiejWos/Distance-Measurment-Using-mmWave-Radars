import logging
import os
import sys
import threading
from pathlib import Path
from enum import Enum


class DCA1000CmdCode(Enum):
    CAPTURE_DONE = 0x08
    SYSTEM_ERROR = 0x0A
    SUCCESS_STATUS = 0
    FAILURE_STATUS = -1


if getattr(sys, "frozen", False):
    _base = Path(sys._MEIPASS)
    os.add_dll_directory(str(_base))
    import importlib
    radar_api = importlib.import_module("radar_api")
else:
    _TI_EXT_DIR = Path(__file__) \
        .resolve().parents[2] / "libs" / "dca1000" / "ti-external"
    if _TI_EXT_DIR.is_dir():
        os.add_dll_directory(str(_TI_EXT_DIR))
    from . import radar_api


class DCA1000:
    def __init__(self, logger: logging.Logger):
        self.radar_api = radar_api
        self.logger = logger
        self.record_done = threading.Event()

    def _event_callback(self, cmd: int, status: int) -> None:
        if cmd == DCA1000CmdCode.SYSTEM_ERROR.value \
                and status == DCA1000CmdCode.CAPTURE_DONE.value:
            self.logger.debug("Record-done callback fired")
            self.record_done.set()

    def init(self) -> bool:
        ret = self.radar_api.dca1000_config_init()
        if ret != DCA1000CmdCode.SUCCESS_STATUS.value:
            self.logger.error("Failed to initialize DCA1000")
            return False
        self.radar_api.register_event_callback(self._event_callback)
        return True

    def set_file_base_path(self, path: str) -> bool:
        ret = self.radar_api.dca_set_file_base_path(path)
        if ret != DCA1000CmdCode.SUCCESS_STATUS.value:
            self.logger.error("Failed to set file base path")
            return False
        return True

    def set_file_prefix(self, prefix: str) -> bool:
        ret = self.radar_api.dca_set_file_prefix(prefix)
        if ret != DCA1000CmdCode.SUCCESS_STATUS.value:
            self.logger.error("Failed to set file prefix")
            return False
        return True

    def trigger_record(self) -> bool:
        self.record_done.clear()
        ret = self.radar_api.dca_trigger_record()
        if ret != DCA1000CmdCode.SUCCESS_STATUS.value:
            self.logger.error("Failed to trigger record")
            return False
        return True

    def stop_record(self) -> bool:
        ret = self.radar_api.dca_stop_record()
        if ret != DCA1000CmdCode.SUCCESS_STATUS.value:
            self.logger.error("Failed to stop record")
            return False
        return True

    def wait_for_record_done(self, timeout: float) -> bool:
        done = self.record_done.wait(timeout)
        self.record_done.clear()
        return done

    def deinit(self) -> None:
        self.radar_api.dca1000_deinit()
        self.record_done.clear()
