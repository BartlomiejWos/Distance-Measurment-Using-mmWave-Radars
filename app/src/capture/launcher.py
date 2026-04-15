import argparse
import logging
import subprocess
import sys
from pathlib import Path

from cli import AppArgs
from cfg.types.radar import RadarCfg
from cfg.frame_builder import build_sfcw_frames, build_fmcw_frames

from . import utils
from . import record


def _build_frames(cfg: RadarCfg,
                  logger: logging.Logger) -> list[list[str]] | None:
    if cfg.fmcw_cfg is not None:
        frames = build_fmcw_frames(cfg, logger)
    else:
        frames = build_sfcw_frames(cfg, logger)
    if frames is None:
        return None
    logger.info("Builder produced %d frame(s)", len(frames))
    return frames


def _build_base_cmd(args: AppArgs) -> list[str]:
    if getattr(sys, "frozen", False):
        base_cmd = [sys.executable, "--_subprocess"]
    else:
        base_cmd = [sys.executable, "-m", "entrypoint", "--_subprocess"]
    if args.verbose:
        base_cmd.append("-v")
    return base_cmd


def _spawn_warmup(base_cmd: list[str], cfg: RadarCfg,
                  logger: logging.Logger) -> int:
    logger.info("Spawning warmup subprocess")
    rc = subprocess.run([*base_cmd, "--mode", "warmup",
                         "--com-port", cfg.com_port,
                         "--output-path", cfg.output_path]).returncode
    if rc != 0:
        logger.error("Warmup failed (rc=%d)", rc)
        return rc
    record.warmup_cleanup(Path(cfg.output_path), logger)
    return 0


def _spawn_frames(base_cmd: list[str], frames: list[list[str]],
                  file_prefix: str, capture_dir: Path, com_port: str,
                  logger: logging.Logger) -> int:
    for i, commands in enumerate(frames):
        prefix = f"{file_prefix}_f{i:03d}"
        frame_file = utils.write_frame_file(commands, str(capture_dir), com_port)
        cmd = [*base_cmd, "--mode", "capture",
               "--frame-file", frame_file, "--prefix", prefix]
        logger.info("Spawning capture subprocess for %s", prefix)
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            logger.error("Capture %s failed (rc=%d)", prefix, rc)
            return rc
    return 0


def run(args: AppArgs, cfg: RadarCfg, capture_dir: Path,
        logger: logging.Logger) -> int:
    """Multi-frame capture — one subprocess per frame for clean DLL state."""
    frames = _build_frames(cfg, logger)
    if frames is None:
        return 1
    base_cmd = _build_base_cmd(args)

    rc = _spawn_warmup(base_cmd, cfg, logger)
    if rc != 0:
        return rc

    rc = _spawn_frames(base_cmd, frames, args.file_prefix,
                       capture_dir, cfg.com_port, logger)
    if rc != 0:
        return rc

    if cfg.sfcw_cfg is not None:
        utils.save_metadata_sfcw(cfg, len(frames), capture_dir)
    logger.info("All %d frames captured successfully", len(frames))
    return 0


def subprocess_entry(argv: list[str], logger: logging.Logger) -> int:
    """Entry point for subprocess re-invocation via --_subprocess."""
    idx = argv.index("--_subprocess")
    sub_argv = argv[idx + 1:]
    parser = argparse.ArgumentParser(prog="app_runner_subprocess")
    parser.add_argument("--mode", required=True, choices=["warmup", "capture"])
    parser.add_argument("--com-port")
    parser.add_argument("--output-path")
    parser.add_argument("--frame-file")
    parser.add_argument("--prefix")
    parser.add_argument("-v", dest="verbose", action="store_true")  # parsed by main()
    ns = parser.parse_args(sub_argv)
    if ns.mode == "warmup":
        if not ns.com_port or not ns.output_path:
            parser.error("--com-port and --output-path required for warmup mode")
        return record.warmup_frame(ns.com_port, ns.output_path, logger)
    else:
        if not ns.frame_file or not ns.prefix:
            parser.error("--frame-file and --prefix required for capture mode")
        return record.single_frame(ns.frame_file, ns.prefix, logger)
