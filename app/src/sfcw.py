import os, subprocess, sys, time
from pathlib import Path

PY  = sys.executable
APP = Path(__file__).resolve().parent / "one_capture.py"

def run(prefix: str):
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"

    print(f"\n==> Start Recording With Prefix: {prefix}")
    cp = subprocess.run(
        [PY, str(APP), prefix],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
        env=env,
    )
    if cp.stdout:
        print(cp.stdout, end="")
    if cp.stderr:
        print(cp.stderr, file=sys.stderr, end="")

if __name__ == "__main__":
    try:
        run("sfcw_multi_chirp")
    except subprocess.CalledProcessError as e:
        if e.stdout:
            print(e.stdout, end="")
        if e.stderr:
            print(e.stderr, file=sys.stderr, end="")
        sys.exit(e.returncode)
