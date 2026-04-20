#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$APP_DIR"

# Run from an MSYS2 MinGW64 shell — pyinstaller must be installed under
# /mingw64/bin/python so the bundled CPython matches radar_api.pyd's ABI.
#
# Layout (everything stays inside build/):
#   build/radar_app.spec  (tracked, hand-edited)
#   build/work/           (intermediates)
#   build/dist/           (final single-file exe)
pyinstaller build/radar_app.spec \
    --workpath build/work \
    --distpath build/dist \
    --noconfirm "$@"
