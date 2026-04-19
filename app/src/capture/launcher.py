import argparse
import logging
import subprocess
import sys
import time
from pathlib import Path

from cli import AppArgs
from cfg.types.radar import RadarCfg
from cfg.frame_builder import build_sfcw_frames, build_fmcw_frames

from . import record
from . import utils


def _build_cfg_frames(cfg: RadarCfg,
                      logger: logging.Logger) -> list[list[str]] | None:
    if cfg.fmcw_cfg is not None:
        frames = build_fmcw_frames(cfg, logger)
    else:
        frames = build_sfcw_frames(cfg, logger)
    if frames is None:
        return None
    return frames


def _build_base_cmd(args: AppArgs) -> list[str]:
    if getattr(sys, "frozen", False):
        base_cmd = [sys.executable, "--_subprocess"]
    else:
        base_cmd = [sys.executable, "-m", "entrypoint", "--_subprocess"]
    if args.verbose:
        base_cmd.append("-v")
    return base_cmd


def _spawn_frame_process(base_cmd: list[str],
                         frame_file: str,
                         prefix: str,
                         idx: int,
                         total: int,
                         logger: logging.Logger) -> int:
    logger.info(f"=== Frame {idx}/{total}: {prefix} ===")
    rc = subprocess.run(
        [*base_cmd, "--frame-file", frame_file, "--prefix", prefix]
    ).returncode
    if rc != utils.ReturnCode.SUCCESS.value:
        logger.error(f"[{prefix}] subprocess failed (rc={rc})")
    return rc


def _spawn_warmup_process(base_cmd: list[str],
                          logger: logging.Logger) -> int:
    logger.info("[warmup] spawning subprocess")
    rc = subprocess.run([*base_cmd, "--warmup"]).returncode
    if rc != utils.ReturnCode.SUCCESS.value:
        logger.error(f"[warmup] subprocess failed (rc={rc})")
    return rc


def run(args: AppArgs,
        cfg: RadarCfg,
        capture_dir: Path,
        logger: logging.Logger) -> int:
    t0 = time.monotonic()
    mode = "FMCW" if cfg.fmcw_cfg is not None else "SFCW"
    utils.banner(f"MEASUREMENT: {args.file_prefix} | {mode}")
    frames = _build_cfg_frames(cfg, logger)
    if frames is None:
        return utils.ReturnCode.FAILURE.value
    logger.info(f"output: {capture_dir}")
    base_cmd = _build_base_cmd(args)
    utils.banner("WARMUP")
    rc = _spawn_warmup_process(base_cmd, logger)
    if rc != utils.ReturnCode.SUCCESS.value:
        return rc
    record.warmup_cleanup(Path(capture_dir), logger)
    utils.banner("CAPTURE")
    total = len(frames)
    for i, commands in enumerate(frames, start=1):
        prefix = f"{args.file_prefix}_f{i - 1:03d}"
        frame_file = utils.write_frame_file(commands)
        rc = _spawn_frame_process(base_cmd,
                                  frame_file,
                                  prefix,
                                  i,
                                  total,
                                  logger)
        if rc != utils.ReturnCode.SUCCESS.value:
            return rc
    if cfg.sfcw_cfg is not None:
        utils.save_metadata_sfcw(cfg, capture_dir)
    else:
        utils.save_metadata_fmcw(cfg, capture_dir)
    elapsed = time.monotonic() - t0
    utils.banner(
        f"COMPLETE: {total}/{total} raw frame files | elapsed {elapsed:.1f}s"
    )
    return utils.ReturnCode.SUCCESS.value


def handle_child_process(argv: list[str],
                         logger: logging.Logger) -> int:
    idx = argv.index("--_subprocess")
    sub_argv = argv[idx + 1:]
    parser = argparse.ArgumentParser(prog="app_runner_subprocess")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--warmup", action="store_true")
    group.add_argument("--frame-file")
    parser.add_argument("--prefix")
    parser.add_argument("-v", dest="verbose", action="store_true")
    ns = parser.parse_args(sub_argv)
    if ns.warmup:
        return record.warmup_frame(logger)
    if not ns.prefix:
        parser.error("--prefix required with --frame-file")
    return record.single_frame(ns.frame_file, ns.prefix, logger)
