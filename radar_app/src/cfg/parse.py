import logging
from typing import Any
from pathlib import Path
import json5

from cfg import types


def _load_json5(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data: dict[str, Any] = json5.load(f)
    return data


def _parse_channel_cfg(json_cfg: dict[str, Any],
                       logger: logging.Logger
                       ) -> types.channel.ChannelCfg | None:
    dfe_output_mode = types.channel.DFEOutputMode.from_int(
        json_cfg.get("dfe_output_mode")
    )
    if dfe_output_mode is None:
        logger.error("channel_cfg.dfe_output_mode: expected 1 or 3")
        return None
    rx_channel_mask = types.channel.RxChannelMask.from_int(
        json_cfg.get("rx_channel_mask")
    )
    if rx_channel_mask is None:
        logger.error("channel_cfg.rx_channel_mask: expected 1, 3, 7 or 15")
        return None
    tx_channel_mask = types.channel.TxChannelMask.from_int(
        json_cfg.get("tx_channel_mask")
    )
    if tx_channel_mask is None:
        logger.error("channel_cfg.tx_channel_mask: expected 1 (TX1 only)")
        return None
    return types.channel.ChannelCfg(
        dfe_output_mode=dfe_output_mode,
        rx_channel_mask=rx_channel_mask,
        tx_channel_mask=tx_channel_mask,
    )


def _parse_adc_cfg(json_cfg: dict[str, Any],
                   logger: logging.Logger) -> types.adc.ADCCfg | None:
    adc_output_fmt = types.adc.ADCOutputFmt.from_int(
        json_cfg.get("adc_output_fmt")
    )
    if adc_output_fmt is None:
        logger.error("adc_cfg.adc_output_fmt: expected 1 or 2")
        return None
    low_power_adc_mode = types.adc.LowPowerAdcMode.from_int(
        json_cfg.get("low_power_adc_mode")
    )
    if low_power_adc_mode is None:
        logger.error("adc_cfg.low_power_adc_mode: expected 0 or 1 ")
        return None
    return types.adc.ADCCfg(
        adc_output_fmt=adc_output_fmt,
        low_power_adc_mode=low_power_adc_mode,
    )


def _parse_profile_cfg(json_cfg: dict[str, Any],
                       logger: logging.Logger
                       ) -> types.profile.ProfileCfg | None:
    idle_time = types.profile.IdleTimeUsec.from_float(
        json_cfg.get("idle_time")
    )
    if idle_time is None:
        logger.error("profile_cfg.idle_time: expected float > 0 [usec]")
        return None
    adc_start_time = types.profile.AdcStartTimeUsec.from_float(
        json_cfg.get("adc_start_time")
    )
    if adc_start_time is None:
        logger.error("profile_cfg.adc_start_time: expected float > 0 [usec]")
        return None
    ramp_end_time = types.profile.RampEndTimeUsec.from_float(
        json_cfg.get("ramp_end_time")
    )
    if ramp_end_time is None:
        logger.error("profile_cfg.ramp_end_time: expected float > 0 [usec]")
        return None
    tx_out_power = types.profile.TxOutPower.from_int(
        json_cfg.get("tx_out_power")
    )
    if tx_out_power is None:
        logger.error("profile_cfg.tx_out_power: expected int >= 0")
        return None
    tx_phase_shifter = types.profile.TxPhaseShifter.from_int(
        json_cfg.get("tx_phase_shifter")
    )
    if tx_phase_shifter is None:
        logger.error("profile_cfg.tx_phase_shifter: expected int >= 0")
        return None
    freq_slope_const = types.profile.FreqSlopeConst.from_float(
        json_cfg.get("freq_slope_const")
    )
    if freq_slope_const is None:
        logger.error("profile_cfg.freq_slope_const: "
                     "expected float >= 0 [MHz/usec]")
        return None
    num_adc_samples = types.profile.NumAdcSamples.from_int(
        json_cfg.get("num_adc_samples")
    )
    if num_adc_samples is None:
        logger.error("profile_cfg.num_adc_samples: expected even int > 0")
        return None
    hpf_corner_freq1 = types.profile.HpfCornerFreq1.from_int(
        json_cfg.get("hpf_corner_freq1")
    )
    if hpf_corner_freq1 is None:
        logger.error("profile_cfg.hpf_corner_freq1: "
                     "expected 0-3 (175/235/350/700 kHz)")
        return None
    hpf_corner_freq2 = types.profile.HpfCornerFreq2.from_int(
        json_cfg.get("hpf_corner_freq2")
    )
    if hpf_corner_freq2 is None:
        logger.error("profile_cfg.hpf_corner_freq2: "
                     "expected 0-3 (350 kHz/700 kHz/1.4/2.8 MHz)")
        return None
    dig_out_sample_rate = types.profile.DigOutSampleRate.from_int(
        json_cfg.get("dig_out_sample_rate")
    )
    if dig_out_sample_rate is None:
        logger.error("profile_cfg.dig_out_sample_rate: expected int > 0 [ksps]")
        return None
    tx_start_time = types.profile.TxStartTimeUsec.from_float(
        json_cfg.get("tx_start_time")
    )
    if tx_start_time is None:
        logger.error("profile_cfg.tx_start_time: expected float > 0 [usec]")
        return None
    rx_gain = types.profile.RxGain.from_int(
        json_cfg.get("rx_gain")
    )
    if rx_gain is None:
        logger.error("profile_cfg.rx_gain: expected int >= 0 [dB]")
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


def _parse_frame_cfg(frame_obj: dict[str, Any],
                     mode: str,
                     logger: logging.Logger
                     ) -> types.frame.FrameCfg | None:
    num_loops = types.frame.NumLoops.from_int(frame_obj.get("num_loops"))
    if num_loops is None:
        logger.error(f"{mode}.frame.num_loops: expected int 1-255")
        return None
    num_frames = types.frame.NumFrames.from_int(frame_obj.get("num_frames"))
    if num_frames is None:
        logger.error(f"{mode}.frame.num_frames: expected int 1-65535")
        return None
    frame_periodicity_ms = types.frame.FramePeriodicityMs.from_float(
        frame_obj.get("frame_periodicity_ms")
    )
    if frame_periodicity_ms is None:
        logger.error(f"{mode}.frame.frame_periodicity_ms: "
                     "expected float > 0 [ms]")
        return None
    return types.frame.FrameCfg(
        num_loops=num_loops,
        num_frames=num_frames,
        frame_periodicity_ms=frame_periodicity_ms,
    )


def _parse_sfcw_cfg(json_cfg: dict[str, Any],
                    logger: logging.Logger) -> types.sfcw.SfcwCfg | None:
    start = types.profile.StartFreqGHz.from_float(
        json_cfg.get("start_freq_Ghz")
    )
    if start is None:
        logger.error("sfcw_cfg.start_freq_Ghz: expected 77.5-81 [GHz]")
        return None
    end = types.profile.EndFreqGHz.from_float(
        json_cfg.get("end_freq_Ghz")
    )
    if end is None:
        logger.error("sfcw_cfg.end_freq_Ghz: expected 77.5-81 [GHz]")
        return None
    df = types.sfcw.DfHz.from_float(
        json_cfg.get("df_hz")
    )
    if df is None:
        logger.error("sfcw_cfg.df_hz: expected float > 0 [Hz]")
        return None
    frame_obj = json_cfg.get("frame")
    if not isinstance(frame_obj, dict):
        logger.error("sfcw_cfg.frame: missing")
        return None
    frame = _parse_frame_cfg(frame_obj, "sfcw_cfg", logger)
    if frame is None:
        return None
    return types.sfcw.SfcwCfg(
        start_freq_ghz=start,
        end_freq_ghz=end,
        df_hz=df,
        frame=frame,
    )


def _parse_fmcw_cfg(json_cfg: dict[str, Any],
                    logger: logging.Logger
                    ) -> types.fmcw.FmcwCfg | None:
    start_freq_ghz = types.profile.StartFreqGHz.from_float(
        json_cfg.get("start_freq_Ghz")
    )
    if start_freq_ghz is None:
        logger.error("fmcw_cfg.start_freq_Ghz: expected 77.5-81 [GHz]")
        return None
    frame_obj = json_cfg.get("frame")
    if not isinstance(frame_obj, dict):
        logger.error("fmcw_cfg.frame: missing")
        return None
    frame = _parse_frame_cfg(frame_obj, "fmcw_cfg", logger)
    if frame is None:
        return None
    return types.fmcw.FmcwCfg(
        start_freq_hz=start_freq_ghz,
        frame=frame,
    )


def _parse_radar_cfg(json_cfg: dict[str, Any],
                     logger: logging.Logger
                     ) -> types.radar.RadarCfg | None:
    has_sfcw = "sfcw_cfg" in json_cfg
    has_fmcw = "fmcw_cfg" in json_cfg
    if has_sfcw and has_fmcw:
        logger.error("Config has both sfcw_cfg and fmcw_cfg"
                     " — use separate files")
        return None
    if not has_sfcw and not has_fmcw:
        logger.error("Config missing sfcw_cfg or fmcw_cfg")
        return None
    output_path = json_cfg.get("output_path")
    if not isinstance(output_path, str):
        logger.error("output_path: expected string")
        return None
    com_port = json_cfg.get("com_port")
    if not isinstance(com_port, str):
        logger.error("com_port: expected string (e.g. 'COM22')")
        return None
    channel_cfg = _parse_channel_cfg(json_cfg["channel_cfg"], logger)
    if channel_cfg is None:
        return None
    adc_cfg = _parse_adc_cfg(json_cfg["adc_cfg"], logger)
    if adc_cfg is None:
        return None
    profile_cfg = _parse_profile_cfg(json_cfg["profile_cfg"], logger)
    if profile_cfg is None:
        return None
    sfcw_cfg: types.sfcw.SfcwCfg | None = None
    if has_sfcw:
        sfcw_cfg = _parse_sfcw_cfg(json_cfg["sfcw_cfg"], logger)
        if sfcw_cfg is None:
            return None
    fmcw_cfg: types.fmcw.FmcwCfg | None = None
    if has_fmcw:
        fmcw_cfg = _parse_fmcw_cfg(json_cfg["fmcw_cfg"], logger)
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


def load_cfg(path: str | Path,
             logger: logging.Logger) -> types.radar.RadarCfg | None:
    json_cfg = _load_json5(path)
    radar_cfg = _parse_radar_cfg(json_cfg, logger)
    if radar_cfg is None:
        return None
    return radar_cfg
