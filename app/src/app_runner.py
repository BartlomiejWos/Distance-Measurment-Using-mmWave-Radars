import argparse
import logging
import numpy as np
import subprocess
import sys
import time

from src.hwctl import HwCtl
from src.hwctl.dca1000 import DCA1000
from src.hwctl.uart import PySerialUart
from src.cli.args import AppArgs
from src.cfg.parse import load_cfg
from src.cfg.radar import RadarCfg


def _build_logger(verbose: bool = False) -> logging.Logger:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return logging.getLogger("radar_app")


def _prepare_commands(cfg: RadarCfg) -> list[str]:
    ch = cfg.channel_cfg
    adc = cfg.adc_cfg
    commands = [
        "flushCfg",
        f"dfeDataOutputMode {ch.dfe_output_mode.to_int()}",
        f"channelCfg {ch.rx_channel_mask.to_int()} {ch.tx_channel_mask.to_int()} 0",
        f"adcCfg {adc.num_adc_bits.to_int()} {adc.adc_output_fmt.to_int()}",
        "adcbufCfg -1 0 1 1 1",
        "lowPower 0 0",
        "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",
    ]
    N = 20          # liczba chirpów
    df = 1e6        # krok 1 MHz
    f0 = 0          # początek zakresu
    fmax = 20e6     # koniec zakresu

    freq_vars = np.arange(f0, fmax, df)
    for i, f_hz in enumerate(freq_vars):
        commands.append(f"chirpCfg {i} {i} 0 {f_hz:.0f} 0 0 0 {ch.tx_channel_mask.to_int()}")

    # Ramka obejmuje wszystkie chirpy
    commands += [
        f"frameCfg 0 {N-1} 1 1 50 1 0",
        "lvdsStreamCfg -1 0 1 0",
    ]
    return commands


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


def _hw_cfg(hw: HwCtl, cfg: RadarCfg, prefix: str,
             log: logging.Logger) -> None:
    """Configure radar via UART and set DCA1000 output path/prefix."""
    commands = _prepare_commands(cfg)
    for cmd in commands:
        hw.uart.send(cmd)
    hw.radar_api.set_file_base_path(cfg.output_path)
    hw.radar_api.set_file_prefix(prefix)
    log.debug("Radar configured, output → %s/%s*", cfg.output_path, prefix)


def _single_capture(config: str, prefix: str, verbose: bool = False) -> int:
    log = _build_logger(verbose)
    log.info("=== Frame: %s ===", prefix)
    try:
        hw = HwCtl(
            uart=PySerialUart(port="COM22", logger=log),
            radar_api=DCA1000(logger=log),
            logger=log,
        )
        cfg = load_cfg(config)

        hw.radar_api.init()
        hw.uart.connect()
        try:
            _hw_cfg(hw, cfg, prefix, log)
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


def run(args: AppArgs, suffixes: list[str] | None = None) -> int:
    """Multi-frame capture — one subprocess per frame for clean DLL state."""
    if suffixes is None:
        suffixes = [""]
    log = _build_logger(args.verbose)

    if getattr(sys, "frozen", False):
        # PyInstaller .exe — sys.executable is the app itself
        base_cmd = [sys.executable, "--_subprocess"]
    else:
        # Normal Python — need script path (sys.argv[0] = main.py)
        base_cmd = [sys.executable, sys.argv[0], "--_subprocess"]
    if args.verbose:
        base_cmd.append("-v")

    # Warmup subprocess
    log.info("Spawning warmup subprocess")
    rc = subprocess.run([*base_cmd, "--mode", "warmup"]).returncode
    if rc != 0:
        log.error("Warmup failed (rc=%d)", rc)
        return rc

    # One subprocess per frame
    for suffix in suffixes:
        prefix = f"{args.file_prefix}{suffix}"
        cmd = [*base_cmd, "--mode", "capture",
               "--config", args.config, "--prefix", prefix]
        log.info("Spawning capture subprocess for %s", prefix)
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            log.error("Capture %s failed (rc=%d)", prefix, rc)
            return rc

    log.info("All frames captured successfully")
    return 0


def subprocess_entry(argv: list[str]) -> int:
    """Entry point for subprocess re-invocation via --_subprocess."""
    # Strip everything before --_subprocess (e.g. the script name)
    idx = argv.index("--_subprocess")
    sub_argv = argv[idx + 1:]

    parser = argparse.ArgumentParser(prog="app_runner_subprocess")
    parser.add_argument("--mode", required=True, choices=["warmup", "capture"])
    parser.add_argument("--config")
    parser.add_argument("--prefix")
    parser.add_argument("-v", "--verbose", action="store_true")
    ns = parser.parse_args(sub_argv)

    if ns.mode == "warmup":
        return _warmup(ns.verbose)
    else:
        if not ns.config or not ns.prefix:
            parser.error("--config and --prefix required for capture mode")
        return _single_capture(ns.config, ns.prefix, ns.verbose)
