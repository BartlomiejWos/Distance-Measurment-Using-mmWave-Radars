import json
import logging
import os
from pathlib import Path

from . import acquisition
from . import utils


WARMUP_COMMANDS = [
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


def warmup_cleanup(output_dir: Path, logger: logging.Logger) -> None:
    for f in output_dir.glob("_warmup*"):
        f.unlink()
        logger.debug(f"[warmup] removed file: {f.name}")
    log_csv = output_dir / "log.csv"
    if log_csv.exists():
        log_csv.unlink()
        logger.debug("[warmup] removed log.csv")


def single_frame(frame_file: str, prefix: str,
                 logger: logging.Logger) -> int:
    with open(frame_file, "r") as f:
        commands = json.load(f)["commands"]
    com_port = os.environ["RADAR_COM_PORT"]
    output_path = os.environ["RADAR_OUTPUT_PATH"]
    rc = acquisition.run(com_port, commands, output_path, prefix, logger,
                         tag=prefix)
    if rc == utils.ReturnCode.SUCCESS.value:
        bin_path = Path(output_path) / f"{prefix}_Raw_0.bin"
        if bin_path.exists():
            size_mb = bin_path.stat().st_size / (1024 * 1024)
            logger.info(f"[{prefix}] captured -> {bin_path.name} "
                        f"({size_mb:.1f} MB)")
        else:
            logger.warning(f"[{prefix}] capture file not found: {bin_path}")
    return rc


def warmup_frame(logger: logging.Logger) -> int:
    # Runs as a dedicated subprocess (fresh DLL load) to shake out the
    # DCA1000 cold-start state before real captures.
    com_port = os.environ["RADAR_COM_PORT"]
    output_path = os.environ["RADAR_OUTPUT_PATH"]
    rc = acquisition.run(com_port, WARMUP_COMMANDS, output_path,
                         "_warmup", logger, wait_for_done=False,
                         tag="warmup")
    if rc == utils.ReturnCode.SUCCESS.value:
        logger.info(f"[warmup] done (rc={rc})")
    return rc
