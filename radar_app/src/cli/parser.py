import argparse
import logging
from pathlib import Path

from .args import AppArgs

DEFAULT_AUTO_PREFIX = "meas"


def _validate_cfg_path(path: str, logger: logging.Logger) -> bool:
    p = Path(path)
    if p.suffix.lower() != ".json5":
        logger.error(f"--cfg-file-path must have .json5 extension: {path}")
        return False
    if not p.exists():
        logger.error(f"Config file not found: {path}")
        return False
    return True


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar_app",
        description="Radar acquisition app using mmWave IWR1843BOOST + DCA1000",
    )
    parser.add_argument(
        "--cfg-file-path",
        default=None,
        help="Path to radar configuration (FMCW or SFCW) stored in json5 file "
             "(default: configuration from cfg/default.py)",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="File prefix for DCA1000 recording file output "
             "(optional if --auto-prefix is used)",
    )
    parser.add_argument(
        "--auto-prefix",
        metavar="BASE",
        default=None,
        help="Auto-increment prefix: e.g. "
             "--auto-prefix meas → meas_000, meas_001, ...",
    )
    parser.add_argument(
        "-v",
        dest="verbose",
        action="store_true",
        help="Enable DEBUG logging (default: INFO)",
    )
    return parser


def parse_args(argv: list[str] | None,
               logger: logging.Logger) -> AppArgs | None:
    parser = _build_arg_parser()
    ns = parser.parse_args(argv)

    if ns.file_prefix is None and ns.auto_prefix is None:
        ns.auto_prefix = DEFAULT_AUTO_PREFIX
    if ns.cfg_file_path is not None and \
            not _validate_cfg_path(ns.cfg_file_path, logger):
        return None
    return AppArgs(
        file_prefix=ns.file_prefix,
        auto_prefix=ns.auto_prefix,
        cfg_file_path=ns.cfg_file_path,
        verbose=ns.verbose,
    )
