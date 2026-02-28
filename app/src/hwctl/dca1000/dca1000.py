import logging
import os
import threading
from pathlib import Path

# ReleaseRF_API.dll lives in libs/dca1000/ — must be registered
# BEFORE importing the .pyd that links against it.
_TI_EXT_DIR = Path(__file__).resolve().parents[3] / "libs" / "dca1000" / "ti-external"
if _TI_EXT_DIR.is_dir():
    os.add_dll_directory(str(_TI_EXT_DIR))

from . import radar_api as _ra  # noqa: E402

class DCA1000:
    def __init__(self, logger: logging.Logger):
        self._ra = _ra
        self._logger = logger
        self._record_done = threading.Event()

    def _event_callback(self, cmd: int, status: int) -> None:
        if cmd == 0x0A and status == 0x8:
            self._logger.debug("Record-done callback fired")
            self._record_done.set()

    def init(self) -> None:
        self._ra.dca1000_config_init()
        self._ra.register_event_callback(self._event_callback)
        self._logger.info("DCA1000 initialised")

    def set_file_base_path(self, path: str) -> None:
        self._ra.dca_set_file_base_path(path)

    def set_file_prefix(self, prefix: str) -> None:
        self._ra.dca_set_file_prefix(prefix)

    def trigger_record(self) -> None:
        self._record_done.clear()
        self._ra.dca_trigger_record()

    def stop_record(self) -> None:
        self._ra.dca_stop_record()

    def wait_for_record_done(self, timeout: float) -> bool:
        done = self._record_done.wait(timeout)
        self._record_done.clear()
        return done

    def deinit(self) -> None:
        self._ra.dca1000_deinit()
        self._record_done.clear()
        self._logger.info("DCA1000 deinitialized")
