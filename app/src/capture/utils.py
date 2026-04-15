import json
import tempfile

from pathlib import Path
from cfg.types.radar import RadarCfg


_BANNER_WIDTH = 100


def banner(msg: str) -> None:
    width: int = _BANNER_WIDTH
    char: str = "="
    pad = width - len(msg) - 2
    if pad < 2:
        print(f"{char} {msg} {char}")
        return
    left = pad // 2
    right = pad - left
    print(f"{char * left} {msg} {char * right}")


def save_metadata_sfcw(cfg: RadarCfg, num_frames: int, capture_dir: Path) -> None:
    """Write metadata.json alongside .bin files for MATLAB postprocessing."""
    assert cfg.sfcw_cfg is not None
    rx_mask = cfg.channel_cfg.rx_channel_mask.to_int()
    metadata = {
        "num_adc_samples": cfg.profile_cfg.num_adc_samples.to_int(),
        "num_adc_bits": 16,
        "num_rx": bin(rx_mask).count("1"),
        "is_real": False,
        "start_freq_hz": cfg.sfcw_cfg.start_freq_hz.to_float(),
        "end_freq_hz": cfg.sfcw_cfg.end_freq_hz.to_float(),
        "df_hz": cfg.sfcw_cfg.df_hz.to_float(),
        "num_frames": num_frames,
    }
    meta_path = capture_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)


def write_frame_file(commands: list[str], output_path: str, com_port: str) -> str:
    data = {"commands": commands, "output_path": output_path, "com_port": com_port}
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", prefix="frame_", delete=False,
    )
    json.dump(data, f)
    f.close()
    return f.name
