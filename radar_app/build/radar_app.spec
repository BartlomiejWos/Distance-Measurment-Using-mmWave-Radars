# -*- mode: python ; coding: utf-8 -*-
from pathlib import Path

APP_DIR = Path(SPECPATH).parent

a = Analysis(
    [str(APP_DIR / "src" / "entrypoint.py")],
    pathex=[str(APP_DIR)],
    binaries=[
        (str(APP_DIR / "src" / "hwctl" / "dca1000" / "radar_api.pyd"), "."),
        (str(APP_DIR / "src" / "libs" / "dca1000" / "ti-external" / "ReleaseRF_API.dll"), "."),
        (str(APP_DIR / "src" / "libs" / "dca1000" / "ti-external" / "RF_API.dll"), "."),
        # NOTE: libpython3.12.dll and its MinGW deps (libgcc_s_seh-1.dll,
        # libwinpthread-1.dll, ...) are auto-bundled by PyInstaller's dep
        # scanner when build.sh is run with MinGW64\bin on PATH.
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="radar_app",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
)
