import argparse
from .args import AppArgs


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar_app",
        description="One capture using mmWave radar + DCA1000",
    )
    parser.add_argument(
        "file_prefix",
        help="File prefix for DCA1000 output",
    )
    parser.add_argument(
        "--config",
        help="Path to radar_cfg.json5",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable DEBUG logging (default: INFO)",
    )
    return parser


def parse_args(argv: list[str] | None = None) -> AppArgs:
    parser = _build_arg_parser()
    ns = parser.parse_args(argv)
    return AppArgs(
        file_prefix=ns.file_prefix,
        config=ns.config,
        verbose=ns.verbose,
    )
