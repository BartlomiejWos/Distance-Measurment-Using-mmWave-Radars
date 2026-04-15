import json
import logging
from pathlib import Path

from . import acquisition
from . import utils


_WARMUP_COMMANDS = [
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 1 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "lowPower 0 0",
    "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "frameCfg 0 0 1 1 50 1 0",
    "lvdsStreamCfg -1 0 1 0",
]


def warmup_frame(com_port: str, output_path: str,
                 logger: logging.Logger) -> int:
    utils.banner("WARMUP: dummy frame (FPGA cold start)")
    rc = acquisition.run(com_port, _WARMUP_COMMANDS,
                         output_path, "_warmup", logger)
    if rc == 0:
        utils.banner("WARMUP done")
    return rc


def warmup_cleanup(output_dir: Path, logger: logging.Logger) -> None:
    for f in output_dir.glob("_warmup*"):
        f.unlink()
        logger.debug("Removed warmup file: %s", f.name)
    log_csv = output_dir / "log.csv"
    if log_csv.exists():
        log_csv.unlink()
        logger.debug("Removed warmup log.csv")


def single_frame(frame_file: str, prefix: str,
                 logger: logging.Logger) -> int:
    utils.banner(f"Frame: {prefix}")
    with open(frame_file, "r") as f:
        data = json.load(f)
    rc = acquisition.run(data["com_port"], data["commands"],
                         data["output_path"], prefix, logger)
    if rc == 0:
        utils.banner(f"Capture done: {prefix}")
    return rc
