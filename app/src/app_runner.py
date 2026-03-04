import argparse
import json
import logging
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from src.hwctl import HwCtl
from src.hwctl.dca1000 import DCA1000
from src.hwctl.uart import PySerialUart
from src.cli.args import AppArgs
from src.cfg.types.radar import RadarCfg
from src.cfg.frame_builder import build_sfcw_frames, build_fmcw_frames


def _build_logger(verbose: bool = False) -> logging.Logger:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return logging.getLogger("radar_app")


_WARMUP_SLEEP_S = 2.0

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


def _cleanup_warmup(output_dir: Path, log: logging.Logger) -> None:
    """Remove warmup artefacts (log.csv, _warmup*.bin) from output dir."""
    for f in output_dir.glob("_warmup*"):
        f.unlink()
        log.debug("Removed warmup file: %s", f.name)
    log_csv = output_dir / "log.csv"
    if log_csv.exists():
        log_csv.unlink()
        log.debug("Removed warmup log.csv")


def _warmup(verbose: bool = False) -> int:
    log = _build_logger(verbose)
    log.info("=== WARMUP: dummy frame (FPGA cold start) ===")
    try:
        hw = HwCtl(
            uart=PySerialUart(port="COM22", logger=log),
            radar_api=DCA1000(logger=log),
            logger=log,
        )
        hw.radar_api.init()
        hw.uart.connect()
        try:
            for cmd in _WARMUP_COMMANDS:
                hw.uart.send(cmd)
            hw.radar_api.set_file_prefix("_warmup")
            hw.radar_api.trigger_record()
            hw.uart.send("sensorStart")
            time.sleep(_WARMUP_SLEEP_S)
            hw.radar_api.stop_record()
        finally:
            hw.uart.close()
            hw.radar_api.deinit()
        log.info("=== WARMUP done ===")
        return 0
    except Exception:
        log.exception("Warmup failed")
        return 1


def _write_frame_file(commands: list[str], output_path: str) -> str:
    """Write frame commands to a temp JSON file. Return path."""
    data = {"commands": commands, "output_path": output_path}
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", prefix="frame_", delete=False,
    )
    json.dump(data, f)
    f.close()
    return f.name


def _hw_cfg(hw: HwCtl, commands: list[str], output_path: str,
            prefix: str, log: logging.Logger) -> None:
    """Configure radar via UART and set DCA1000 output path/prefix."""
    for cmd in commands:
        hw.uart.send(cmd)
    hw.radar_api.set_file_base_path(output_path)
    hw.radar_api.set_file_prefix(prefix)
    log.debug("Radar configured, output → %s/%s*", output_path, prefix)


def _single_capture(frame_file: str, prefix: str,
                    verbose: bool = False) -> int:
    log = _build_logger(verbose)
    log.info("=== Frame: %s ===", prefix)
    try:
        with open(frame_file, "r") as f:
            data = json.load(f)
        commands = data["commands"]
        output_path = data["output_path"]

        hw = HwCtl(
            uart=PySerialUart(port="COM22", logger=log),
            radar_api=DCA1000(logger=log),
            logger=log,
        )

        hw.radar_api.init()
        hw.uart.connect()
        try:
            _hw_cfg(hw, commands, output_path, prefix, log)
            hw.radar_api.trigger_record()
            hw.uart.send("sensorStart")
            if not hw.radar_api.wait_for_record_done(timeout=20.0):
                log.error("Capture timed out for %s", prefix)
                return 1
            hw.radar_api.stop_record()
            log.info("Capture done: %s", prefix)
        finally:
            hw.uart.close()
            hw.radar_api.deinit()
        return 0
    except Exception:
        log.exception("Capture failed")
        return 1


def _save_metadata_sfcw(cfg: RadarCfg, num_frames: int, capture_dir: Path) -> None:
    """Write metadata.json alongside .bin files for MATLAB postprocessing."""
    rx_mask = int(cfg.channel_cfg.rx_channel_mask)
    metadata = {
        "num_adc_samples": int(cfg.profile_cfg.num_adc_samples),
        "num_adc_bits": 16,
        "num_rx": bin(rx_mask).count("1"),
        "is_real": False,
        "start_freq_hz": cfg.sfcw_cfg.start_freq_hz,
        "end_freq_hz": cfg.sfcw_cfg.end_freq_hz,
        "df_hz": cfg.sfcw_cfg.df_hz,
        "num_frames": num_frames,
    }
    meta_path = capture_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)


def run(args: AppArgs, cfg: RadarCfg, capture_dir: Path) -> int:
    """Multi-frame capture — one subprocess per frame for clean DLL state."""
    log = _build_logger(args.verbose)

    if cfg.fmcw_cfg is not None:
        frames = build_fmcw_frames(cfg)
    else:
        frames = build_sfcw_frames(cfg)
    log.info("Builder produced %d frame(s)", len(frames))

    if getattr(sys, "frozen", False):
        base_cmd = [sys.executable, "--_subprocess"]
    else:
        base_cmd = [sys.executable, sys.argv[0], "--_subprocess"]
    if args.verbose:
        base_cmd.append("-v")

    # Warmup subprocess
    log.info("Spawning warmup subprocess")
    rc = subprocess.run([*base_cmd, "--mode", "warmup"]).returncode
    if rc != 0:
        log.error("Warmup failed (rc=%d)", rc)
        return rc

    _cleanup_warmup(Path(cfg.output_path), log)

    # One subprocess per frame
    for i, commands in enumerate(frames):
        prefix = f"{args.file_prefix}_f{i:03d}"
        frame_file = _write_frame_file(commands, str(capture_dir))
        cmd = [*base_cmd, "--mode", "capture",
               "--frame-file", frame_file, "--prefix", prefix]
        log.info("Spawning capture subprocess for %s", prefix)
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            log.error("Capture %s failed (rc=%d)", prefix, rc)
            return rc

    _save_metadata_sfcw(cfg, len(frames), capture_dir)
    log.info("All %d frames captured successfully", len(frames))
    return 0


def subprocess_entry(argv: list[str]) -> int:
    """Entry point for subprocess re-invocation via --_subprocess."""
    idx = argv.index("--_subprocess")
    sub_argv = argv[idx + 1:]

    parser = argparse.ArgumentParser(prog="app_runner_subprocess")
    parser.add_argument("--mode", required=True, choices=["warmup", "capture"])
    parser.add_argument("--frame-file")
    parser.add_argument("--prefix")
    parser.add_argument("-v", "--verbose", action="store_true")
    ns = parser.parse_args(sub_argv)

    if ns.mode == "warmup":
        return _warmup(ns.verbose)
    else:
        if not ns.frame_file or not ns.prefix:
            parser.error("--frame-file and --prefix required for capture mode")
        return _single_capture(ns.frame_file, ns.prefix, ns.verbose)
