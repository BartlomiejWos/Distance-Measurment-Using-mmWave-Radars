from typing import Any
from pathlib import Path
import json5

from src.cfg.channel import DFEOutputMode, RxChannelMask, TxChannelMask, ChannelCfg
from src.cfg.adc import NumADCBits, ADCOutputFmt, ADCCfg
from src.cfg.radar import RadarCfg


def _load_json5(path: str | Path) ->  dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json5.load(f)
    return data

def _parse_channel_cfg(json_cfg: dict[str, Any]
                       ) -> ChannelCfg | None:
    dfe_output_mode = DFEOutputMode.from_int(
        json_cfg.get("dfe_output_mode")
    )
    if dfe_output_mode is None:
        return None

    rx_channel_mask = RxChannelMask.from_int(
        json_cfg.get("rx_channel_mask")
    )
    if rx_channel_mask is None:
        return None

    tx_channel_mask = TxChannelMask.from_int(
        json_cfg.get("tx_channel_mask")
    )
    if tx_channel_mask is None:
        return None

    return ChannelCfg(
        dfe_output_mode=dfe_output_mode,
        rx_channel_mask=rx_channel_mask,
        tx_channel_mask=tx_channel_mask,
    )

def _parse_adc_cfg(json_cfg: dict[str, Any]
                   ) -> ADCCfg | None:
    num_adc_bits = NumADCBits.from_int(
        json_cfg.get("num_adc_bits")
    )
    if num_adc_bits is None:
        return None

    adc_output_fmt = ADCOutputFmt.from_int(
        json_cfg.get("adc_output_fmt")
    )
    if adc_output_fmt is None:
        return None

    return ADCCfg(
        num_adc_bits=num_adc_bits,
        adc_output_fmt=adc_output_fmt,
    )


def _parse_radar_cfg(json_cfg: dict[str, Any]) ->  RadarCfg | None:

    output_path = json_cfg.get("output_path")
    if not isinstance(output_path, str):
        return None

    channel_cfg = _parse_channel_cfg(json_cfg["channel_cfg"])
    if channel_cfg is None:
        return None

    adc_cfg = _parse_adc_cfg(json_cfg["adc_cfg"])
    if adc_cfg is None:
        return None

    return RadarCfg(
        output_path=output_path,
        channel_cfg=channel_cfg,
        adc_cfg=adc_cfg,
    )


def load_cfg(path: str | Path) ->  RadarCfg | None:
    json_cfg = _load_json5(path)
    radar_cfg = _parse_radar_cfg(json_cfg)
    if radar_cfg is None:
        return None
    return radar_cfg
