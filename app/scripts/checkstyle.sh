#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$APP_DIR"

TARGETS=(
    src/hwctl
    src/ports
    src/cli
    src/cfg
    src/capture
)

echo "=== mypy ==="
mypy --config-file scripts/mypy.ini "${TARGETS[@]}"

echo "=== flake8 ==="
flake8 --config scripts/.flake8 "${TARGETS[@]}"