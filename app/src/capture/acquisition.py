import logging
import time

from tqdm import tqdm

from hwctl import HwCtl
from hwctl.dca1000 import DCA1000
from hwctl.uart import Uart

from . import utils


_ACQ_TIMEOUT_S = 20.0
_WARMUP_SLEEP_S = 2.0


def _radar_init(hw_ctl: HwCtl) -> bool:
    if not hw_ctl.radar_api.init():
        return False
    if not hw_ctl.uart.connect():
        return False
    return True


def _radar_cfg(hw_ctl: HwCtl,
               commands: list[str],
               output_path: str,
               prefix: str,
               logger: logging.Logger) -> bool:
    # Order matters: UART commands MUST be sent before the DCA1000
    # file path/prefix setters. Reversing the order triggers profileCfg
    # Error -51 from the radar firmware.
    disable_bar = logger.isEnabledFor(logging.DEBUG)
    for cmd in tqdm(commands, desc="Radar Configuration",
                    leave=False, disable=disable_bar,
                    bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}"):
        if not hw_ctl.uart.send(cmd):
            return False
    if not hw_ctl.radar_api.set_file_base_path(output_path):
        return False
    if not hw_ctl.radar_api.set_file_prefix(prefix):
        return False
    return True


def _radar_start_acq(hw_ctl: HwCtl,
                     tag: str,
                     logger: logging.Logger,
                     wait_timeout: float | None) -> bool:
    if not hw_ctl.radar_api.trigger_record():
        return False
    if not hw_ctl.uart.send("sensorStart"):
        return False
    if wait_timeout is not None:
        logger.info(f"[{tag}] sensorStart -> waiting for record-done")
        t0 = time.monotonic()
        if not hw_ctl.radar_api.wait_for_record_done(timeout=wait_timeout):
            logger.error(f"[{tag}] acquisition timed out ({wait_timeout:.1f}s)")
            return False
        logger.info(f"[{tag}] record-done received "
                    f"({time.monotonic() - t0:.2f}s)")
    else:
        logger.info(f"[{tag}] sensorStart -> sleep {_WARMUP_SLEEP_S:.1f}s")
        time.sleep(_WARMUP_SLEEP_S)
    return True


def _radar_stop_acq(hw_ctl: HwCtl) -> bool:
    return hw_ctl.radar_api.stop_record()


def _radar_deinit(hw_ctl: HwCtl) -> None:
    hw_ctl.uart.close()
    hw_ctl.radar_api.deinit()


def run(com_port: str,
        commands: list[str],
        output_path: str,
        prefix: str,
        logger: logging.Logger,
        wait_for_done: bool = True,
        tag: str | None = None) -> int:
    tag = tag or prefix
    try:
        hw_ctl = HwCtl(
            uart=Uart(port=com_port, logger=logger),
            radar_api=DCA1000(logger=logger),
            logger=logger,
        )
        if not _radar_init(hw_ctl):
            logger.error(f"[{tag}] radar init failed")
            return utils.ReturnCode.FAILURE.value
        logger.info(f"[{tag}] DCA1000 + UART ready")
        try:
            logger.info(f"[{tag}] Radar configuration started")
            if not _radar_cfg(hw_ctl, commands, output_path, prefix, logger):
                logger.error(f"[{tag}] Radar configuration failed")
                return utils.ReturnCode.FAILURE.value
            logger.info(f"[{tag}] Radar configuration finished")
            wait_timeout = _ACQ_TIMEOUT_S if wait_for_done else None
            if not _radar_start_acq(hw_ctl, tag, logger, wait_timeout):
                return utils.ReturnCode.FAILURE.value
            if not _radar_stop_acq(hw_ctl):
                logger.error(f"[{tag}] stop_record failed")
                return utils.ReturnCode.FAILURE.value
        finally:
            _radar_deinit(hw_ctl)
        return utils.ReturnCode.SUCCESS.value
    except Exception:
        logger.exception(f"[{tag}] acquisition failed")
        return utils.ReturnCode.FAILURE.value
