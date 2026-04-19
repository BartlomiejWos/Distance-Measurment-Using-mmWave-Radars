import json
import tempfile
from enum import IntEnum
from typing import Any
import re
from pathlib import Path
from colorama import Fore, Style

from cfg.types.radar import RadarCfg


class ReturnCode(IntEnum):
    SUCCESS = 0
    FAILURE = 1


_BANNER_WIDTH = 100
_BANNER_COLOR = Fore.GREEN + Style.BRIGHT


def banner(msg: str) -> None:
    width: int = _BANNER_WIDTH
    char: str = "="
    pad = width - len(msg) - 2
    if pad < 2:
        line = f"{char} {msg} {char}"
    else:
        left = pad // 2
        right = pad - left
        line = f"{char * left} {msg} {char * right}"
    print(f"{_BANNER_COLOR}{line}{Style.RESET_ALL}")


def _common_metadata(cfg: RadarCfg) -> dict[str, Any]:
    rx_mask = cfg.channel_cfg.rx_channel_mask.to_int()
    return {
        "num_adc_samples": cfg.profile_cfg.num_adc_samples.to_int(),
        "num_rx": bin(rx_mask).count("1"),
        "is_real": False,
    }


def save_metadata_sfcw(cfg: RadarCfg, capture_dir: Path) -> None:
    assert cfg.sfcw_cfg is not None
    metadata = {
        **_common_metadata(cfg),
        "start_freq_hz": cfg.sfcw_cfg.start_freq_hz.to_float(),
        "end_freq_hz": cfg.sfcw_cfg.end_freq_hz.to_float(),
        "df_hz": cfg.sfcw_cfg.df_hz.to_float(),
        "num_loops": cfg.sfcw_cfg.frame.num_loops.to_int(),
        "num_frames": cfg.sfcw_cfg.frame.num_frames.to_int(),
        "frame_periodicity_ms": cfg.sfcw_cfg.frame.
        frame_periodicity_ms.to_float(),
    }
    meta_path = capture_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)


def save_metadata_fmcw(cfg: RadarCfg, capture_dir: Path) -> None:
    assert cfg.fmcw_cfg is not None
    metadata = {
        **_common_metadata(cfg),
        "start_freq_hz": cfg.fmcw_cfg.start_freq_hz.to_float(),
        "freq_slope_mhz_per_usec": cfg.profile_cfg.freq_slope_const.to_float(),
        "num_loops": cfg.fmcw_cfg.frame.num_loops.to_int(),
        "num_frames": cfg.fmcw_cfg.frame.num_frames.to_int(),
        "frame_periodicity_ms": cfg.fmcw_cfg.frame.
        frame_periodicity_ms.to_float(),
    }
    meta_path = capture_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)


def write_frame_file(commands: list[str]) -> str:
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", prefix="frame_", delete=False,
    )
    json.dump({"commands": commands}, f)
    f.close()
    return f.name


def next_prefix(base: str, output_dir: Path) -> str:
    pattern = re.compile(rf"^{re.escape(base)}_(\d{{3}})$")
    max_idx = -1
    if output_dir.is_dir():
        for entry in output_dir.iterdir():
            if entry.is_dir():
                m = pattern.match(entry.name)
                if m:
                    max_idx = max(max_idx, int(m.group(1)))
    return f"{base}_{max_idx + 1:03d}"
