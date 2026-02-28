#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$APP_DIR"

echo "=== flake8 ==="
flake8 --config checkstyle/.flake8

echo "=== mypy ==="
mypy --config-file checkstyle/mypy.ini

# echo "=== unittest ==="
# python -m pytest tests/
