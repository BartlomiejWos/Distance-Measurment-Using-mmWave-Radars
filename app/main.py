import sys
from pathlib import Path

from src.cli.parser import parse_args
from src.cfg.parse import load_cfg
from src import app_runner


def main() -> int:
    # Subprocess re-entry: main.exe --_subprocess --mode warmup ...
    if "--_subprocess" in sys.argv:
        return app_runner.subprocess_entry(sys.argv)

    args = parse_args()

    cfg = load_cfg(args.config)
    if cfg is None:
        print(f"Failed to parse config: {args.config}")
        return 1

    capture_dir = Path(cfg.output_path) / args.file_prefix
    capture_dir.mkdir(parents=True, exist_ok=True)

    return app_runner.run(args, cfg, capture_dir)


if __name__ == "__main__":
    raise SystemExit(main())
