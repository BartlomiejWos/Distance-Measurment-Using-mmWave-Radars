from typing import Any
from pathlib import Path
import json5

from cfg import types


def _load_json5(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data: dict[str, Any] = json5.load(f)
    return data


def _parse_channel_cfg(json_cfg: dict[str, Any]
                       ) -> types.channel.ChannelCfg | None:
    dfe_output_mode = types.channel.DFEOutputMode.from_int(
        json_cfg.get("dfe_output_mode")
    )
    if dfe_output_mode is None:
        return None
    rx_channel_mask = types.channel.RxChannelMask.from_int(
        json_cfg.get("rx_channel_mask")
    )
    if rx_channel_mask is None:
        return None
    tx_channel_mask = types.channel.TxChannelMask.from_int(
        json_cfg.get("tx_channel_mask")
    )
    if tx_channel_mask is None:
        return None
    return types.channel.ChannelCfg(
        dfe_output_mode=dfe_output_mode,
        rx_channel_mask=rx_channel_mask,
        tx_channel_mask=tx_channel_mask,
    )


def _parse_adc_cfg(json_cfg: dict[str, Any]) -> types.adc.ADCCfg | None:
    adc_output_fmt = types.adc.ADCOutputFmt.from_int(
        json_cfg.get("adc_output_fmt")
    )
    if adc_output_fmt is None:
        return None
    low_power_adc_mode = types.adc.LowPowerAdcMode.from_int(
        json_cfg.get("low_power_adc_mode")
    )
    if low_power_adc_mode is None:
        return None
    return types.adc.ADCCfg(
        adc_output_fmt=adc_output_fmt,
        low_power_adc_mode=low_power_adc_mode,
    )


def _parse_profile_cfg(json_cfg: dict[str, Any]
                       ) -> types.profile.ProfileCfg | None:
    idle_time = types.profile.IdleTimeUsec.from_float(
        json_cfg.get("idle_time")
    )
    if idle_time is None:
        return None
    adc_start_time = types.profile.AdcStartTimeUsec.from_float(
        json_cfg.get("adc_start_time")
    )
    if adc_start_time is None:
        return None
    ramp_end_time = types.profile.RampEndTimeUsec.from_float(
        json_cfg.get("ramp_end_time")
    )
    if ramp_end_time is None:
        return None
    tx_out_power = types.profile.TxOutPower.from_int(
        json_cfg.get("tx_out_power")
    )
    if tx_out_power is None:
        return None
    tx_phase_shifter = types.profile.TxPhaseShifter.from_int(
        json_cfg.get("tx_phase_shifter")
    )
    if tx_phase_shifter is None:
        return None
    freq_slope_const = types.profile.FreqSlopeConst.from_float(
        json_cfg.get("freq_slope_const")
    )
    if freq_slope_const is None:
        return None
    num_adc_samples = types.profile.NumAdcSamples.from_int(
        json_cfg.get("num_adc_samples")
    )
    if num_adc_samples is None:
        return None
    hpf_corner_freq1 = types.profile.HpfCornerFreq1.from_int(
        json_cfg.get("hpf_corner_freq1")
    )
    if hpf_corner_freq1 is None:
        return None
    hpf_corner_freq2 = types.profile.HpfCornerFreq2.from_int(
        json_cfg.get("hpf_corner_freq2")
    )
    if hpf_corner_freq2 is None:
        return None
    dig_out_sample_rate = types.profile.DigOutSampleRate.from_int(
        json_cfg.get("dig_out_sample_rate")
    )
    if dig_out_sample_rate is None:
        return None
    tx_start_time = types.profile.TxStartTimeUsec.from_float(
        json_cfg.get("tx_start_time")
    )
    if tx_start_time is None:
        return None
    rx_gain = types.profile.RxGain.from_int(
        json_cfg.get("rx_gain")
    )
    if rx_gain is None:
        return None
    return types.profile.ProfileCfg(
        idle_time=idle_time,
        adc_start_time=adc_start_time,
        ramp_end_time=ramp_end_time,
        tx_out_power=tx_out_power,
        tx_phase_shifter=tx_phase_shifter,
        freq_slope_const=freq_slope_const,
        num_adc_samples=num_adc_samples,
        hpf_corner_freq1=hpf_corner_freq1,
        hpf_corner_freq2=hpf_corner_freq2,
        dig_out_sample_rate=dig_out_sample_rate,
        tx_start_time=tx_start_time,
        rx_gain=rx_gain,
    )


def _parse_sfcw_cfg(json_cfg: dict[str, Any]) -> types.sfcw.SfcwCfg | None:
    start = types.sfcw.SfcwStartFreqHz.from_float(
        json_cfg.get("start_freq_hz")
    )
    if start is None:
        return None
    end = types.sfcw.SfcwEndFreqHz.from_float(
        json_cfg.get("end_freq_hz")
    )
    if end is None:
        return None
    df = types.sfcw.DfHz.from_float(
        json_cfg.get("df_hz")
    )
    if df is None:
        return None
    return types.sfcw.SfcwCfg(
        start_freq_hz=start,
        end_freq_hz=end,
        df_hz=df,
    )


def _parse_fmcw_cfg(json_cfg: dict[str, Any]
                    ) -> types.fmcw.FmcwCfg | None:
    start_freq_hz = json_cfg.get("start_freq_hz")
    if not isinstance(start_freq_hz, (int, float)):
        return None
    chirp_obj = json_cfg.get("chirp")
    frame_obj = json_cfg.get("frame")
    if not isinstance(chirp_obj, dict) or not isinstance(frame_obj, dict):
        return None
    start_idx = chirp_obj.get("start_idx")
    end_idx = chirp_obj.get("end_idx")
    if not isinstance(start_idx, int) or not isinstance(end_idx, int):
        return None
    chirp = types.fmcw.ChirpCfg(start_idx=start_idx, end_idx=end_idx)
    chirp_start_idx = frame_obj.get("chirp_start_idx")
    if not isinstance(chirp_start_idx, int):
        return None
    chirp_end_idx = frame_obj.get("chirp_end_idx")
    if not isinstance(chirp_end_idx, int):
        return None
    num_loops = frame_obj.get("num_loops")
    if not isinstance(num_loops, int):
        return None
    num_frames = frame_obj.get("num_frames")
    if not isinstance(num_frames, int):
        return None
    frame_periodicity_ms = frame_obj.get("frame_periodicity_ms")
    if not isinstance(frame_periodicity_ms, (int, float)):
        return None
    frame = types.fmcw.FrameCfg(
        chirp_start_idx=chirp_start_idx,
        chirp_end_idx=chirp_end_idx,
        num_loops=num_loops,
        num_frames=num_frames,
        frame_periodicity_ms=float(frame_periodicity_ms),
    )
    return types.fmcw.FmcwCfg(
        start_freq_hz=float(start_freq_hz),
        chirp=chirp,
        frame=frame,
    )


def _parse_radar_cfg(json_cfg: dict[str, Any]
                     ) -> types.radar.RadarCfg | None:
    has_sfcw = "sfcw_cfg" in json_cfg
    has_fmcw = "fmcw_cfg" in json_cfg

    if has_sfcw and has_fmcw:
        print("[ERROR] Config has both sfcw_cfg and fmcw_cfg"
              " — use separate files")
        return None
    if not has_sfcw and not has_fmcw:
        print("[ERROR] Config missing sfcw_cfg or fmcw_cfg")
        return None

    output_path = json_cfg.get("output_path")
    if not isinstance(output_path, str):
        return None

    com_port = json_cfg.get("com_port")
    if not isinstance(com_port, str):
        return None

    channel_cfg = _parse_channel_cfg(json_cfg["channel_cfg"])
    if channel_cfg is None:
        return None

    adc_cfg = _parse_adc_cfg(json_cfg["adc_cfg"])
    if adc_cfg is None:
        return None

    profile_cfg = _parse_profile_cfg(json_cfg["profile_cfg"])
    if profile_cfg is None:
        return None

    sfcw_cfg: types.sfcw.SfcwCfg | None = None
    if has_sfcw:
        sfcw_cfg = _parse_sfcw_cfg(json_cfg["sfcw_cfg"])
        if sfcw_cfg is None:
            return None

    fmcw_cfg: types.fmcw.FmcwCfg | None = None
    if has_fmcw:
        fmcw_cfg = _parse_fmcw_cfg(json_cfg["fmcw_cfg"])
        if fmcw_cfg is None:
            return None

    return types.radar.RadarCfg(
        output_path=output_path,
        com_port=com_port,
        channel_cfg=channel_cfg,
        adc_cfg=adc_cfg,
        profile_cfg=profile_cfg,
        sfcw_cfg=sfcw_cfg,
        fmcw_cfg=fmcw_cfg,
    )


def load_cfg(path: str | Path) -> types.radar.RadarCfg | None:
    json_cfg = _load_json5(path)
    radar_cfg = _parse_radar_cfg(json_cfg)
    if radar_cfg is None:
        return None
    return radar_cfg
