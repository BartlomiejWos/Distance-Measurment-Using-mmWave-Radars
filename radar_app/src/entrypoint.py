import logging
import os
import sys
from pathlib import Path
import colorama
from colorlog import ColoredFormatter


from cfg.parse import load_cfg, load_default_cfg
from cfg.types.radar import RadarCfg
import cli
import capture


def _build_logger(verbose: bool = False) -> logging.Logger:
    colorama.just_fix_windows_console()
    handler = logging.StreamHandler()
    handler.setFormatter(ColoredFormatter(
        "%(log_color)s%(levelname)s%(reset)s "
        "%(white)s%(name)s%(reset)s: %(message)s",
        log_colors={
            "DEBUG":    "light_black",
            "INFO":     "blue",
            "WARNING":  "yellow",
            "ERROR":    "red",
            "CRITICAL": "bold_red",
        },
    ))
    root = logging.getLogger()
    root.handlers = [handler]
    root.setLevel(logging.DEBUG if verbose else logging.INFO)
    return logging.getLogger("radar_app")


def _set_session_variables(cfg: RadarCfg, capture_dir: Path) -> None:
    os.environ["RADAR_COM_PORT"] = cfg.com_port
    os.environ["RADAR_OUTPUT_PATH"] = str(capture_dir)


def main() -> int:
    verbose = "-v" in sys.argv
    logger = _build_logger(verbose)
    if "--_subprocess" in sys.argv:
        return capture.handle_child_process(sys.argv, logger)
    args = cli.parse_args(sys.argv[1:], logger)
    if args is None:
        return capture.ReturnCode.FAILURE.value
    if args.cfg_file_path is None:
        logger.info("Using default configuration (cfg/default.py)")
        cfg = load_default_cfg(logger)
    else:
        cfg = load_cfg(args.cfg_file_path, logger)
    if cfg is None:
        logger.error(f"Failed to parse config: "
                     f"{args.cfg_file_path or 'default'}")
        return capture.ReturnCode.FAILURE.value
    if args.auto_prefix is not None:
        args.file_prefix = capture.utils.next_prefix(args.auto_prefix,
                                                     Path(cfg.output_path))
    assert args.file_prefix is not None
    capture_dir = Path(cfg.output_path) / args.file_prefix
    capture_dir.mkdir(parents=True, exist_ok=True)
    _set_session_variables(cfg, capture_dir)
    return capture.run(args, cfg, capture_dir, logger)


if __name__ == "__main__":
    raise SystemExit(main())
