import logging

from hwctl import HwCtl
from hwctl.dca1000 import DCA1000
from hwctl.uart import Uart

from . import utils


_ACQ_TIMEOUT_S = 20.0


def _radar_init(hw_ctl: HwCtl, logger: logging.Logger) -> bool:
    utils.banner("Radar initializing started")
    if not hw_ctl.radar_api.init():
        return False
    if not hw_ctl.uart.connect():
        return False
    utils.banner("Radar initialized")
    return True


def _radar_cfg(hw_ctl: HwCtl, commands: list[str], output_path: str,
            prefix: str, logger: logging.Logger) -> bool:
    utils.banner("Radar configuration started")
    if not hw_ctl.radar_api.set_file_base_path(output_path):
        return False
    if not hw_ctl.radar_api.set_file_prefix(prefix):
        return False
    for cmd in commands:
        if not hw_ctl.uart.send(cmd):
            return False
    utils.banner("Radar configured")
    return True


def _radar_start_acq(hw_ctl: HwCtl, logger: logging.Logger,
                    wait_timeout: float | None = None) -> bool:
    if not hw_ctl.radar_api.trigger_record():
        return False
    if not hw_ctl.uart.send("sensorStart"):
        return False
    if wait_timeout is not None:
        if not hw_ctl.radar_api.wait_for_record_done(timeout=wait_timeout):
            logger.error("Acquisition timed out (%.1fs)", wait_timeout)
            return False
    return True


def _radar_stop_acq(hw_ctl: HwCtl) -> bool:
    if not hw_ctl.radar_api.stop_record():
        return False
    return True


def _radar_deinit(hw_ctl: HwCtl, logger: logging.Logger) -> None:
    utils.banner("Radar deinit started")
    hw_ctl.uart.close()
    hw_ctl.radar_api.deinit()
    utils.banner("Radar deinitialized")
    return None


def run(com_port: str, commands: list[str], output_path: str,
        prefix: str, logger: logging.Logger) -> int:
    try:
        hw_ctl = HwCtl(
            uart=Uart(port=com_port, logger=logger),
            radar_api=DCA1000(logger=logger),
            logger=logger,
        )
        if not _radar_init(hw_ctl, logger):
            utils.banner("Radar initializing failed")
            return 1
        try:
            if not _radar_cfg(hw_ctl, commands, output_path, prefix, logger):
                return 1
            if not _radar_start_acq(hw_ctl, logger, wait_timeout=_ACQ_TIMEOUT_S):
                return 1
            if not _radar_stop_acq(hw_ctl):
                return 1
        finally:
            _radar_deinit(hw_ctl, logger)
        return 0
    except Exception:
        logger.exception("Acquisition failed")
        return 1
