from typing import Any
from pathlib import Path
import json5

from . import params
from . import radar_cfg


def _load_json5(path: str | Path) ->  dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json5.load(f)
    return data

def _parse_channel_cfg(json_cfg: dict[str, Any]
                       ) -> radar_cfg.ChannelCfg | None:
    dfe_output_mode = params.DFEOutputMode.from_int(
        json_cfg.get("dfe_output_mode")
    )
    if dfe_output_mode is None:
        return None

    rx_channel_mask = params.RxChannelMask.from_int(
        json_cfg.get("rx_channel_mask")
    )
    if rx_channel_mask is None:
        return None

    tx_channel_mask = params.TxChannelMask.from_int(
        json_cfg.get("tx_channel_mask")
    )
    if tx_channel_mask is None:
        return None

    return radar_cfg.ChannelCfg(
        dfe_output_mode=dfe_output_mode,
        rx_channel_mask=rx_channel_mask,
        tx_channel_mask=tx_channel_mask,
    )

def _parse_radar_cfg(json_cfg: dict[str, Any]) ->  radar_cfg.RadarCfg | None:

    channel_cfg = _parse_channel_cfg(json_cfg["channel_cfg"])
    if channel_cfg is None:
        return None

    return radar_cfg.RadarCfg(
        channel_cfg=channel_cfg
    )


def load_cfg(path: str | Path) ->  radar_cfg.RadarCfg | None:
    json_cfg = _load_json5(path)
    radar_cfg = _parse_radar_cfg(json_cfg)
    if radar_cfg is None:
        return None
    return radar_cfg
