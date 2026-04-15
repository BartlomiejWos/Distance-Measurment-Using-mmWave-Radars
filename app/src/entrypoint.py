import logging
import re
import sys
from pathlib import Path

from cli import parse_args
from cfg.parse import load_cfg
import capture


def build_logger(verbose: bool = False) -> logging.Logger:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return logging.getLogger("radar_app")


def _next_prefix(base: str, output_dir: Path) -> str:
    pattern = re.compile(rf"^{re.escape(base)}_(\d{{3}})$")
    max_idx = -1
    if output_dir.is_dir():
        for entry in output_dir.iterdir():
            if entry.is_dir():
                m = pattern.match(entry.name)
                if m:
                    max_idx = max(max_idx, int(m.group(1)))
    return f"{base}_{max_idx + 1:03d}"

def main() -> int:
    verbose = "-v" in sys.argv
    logger = build_logger(verbose)

    if "--_subprocess" in sys.argv:
        return capture.subprocess_entry(sys.argv, logger)

    args = parse_args(sys.argv[1:], logger)
    if args is None:
        return 1
    cfg = load_cfg(args.cfg_file_path)
    if cfg is None:
        logger.error(f"Failed to parse config: {args.cfg_file_path}")
        return 1

    if args.auto_prefix is not None:
        prefix = _next_prefix(args.auto_prefix, Path(cfg.output_path))
        args.file_prefix = prefix

    capture_dir = Path(cfg.output_path) / args.file_prefix
    capture_dir.mkdir(parents=True, exist_ok=True)

    print(f"Capture prefix: {args.file_prefix}")
    return capture.run(args, cfg, capture_dir, logger)


if __name__ == "__main__":
    raise SystemExit(main())
