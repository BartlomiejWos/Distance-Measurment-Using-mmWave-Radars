import sys

from src.cli.parser import parse_args
from src import app_runner


def main() -> int:
    # Subprocess re-entry: main.exe --_subprocess --mode warmup ...
    if "--_subprocess" in sys.argv:
        return app_runner.subprocess_entry(sys.argv)

    args = parse_args()
    return app_runner.run(args, suffixes=["_1", "_2"])


if __name__ == "__main__":
    raise SystemExit(main())
