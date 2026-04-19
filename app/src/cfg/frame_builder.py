import logging

from .types.radar import RadarCfg
from . import types

_MAX_CHIRPS_PER_FRAME = 512
_MAX_START_FREQ_VAR_HZ = 450e6


def _ghz_to_hz(freq_hz: int) -> int:
    return freq_hz // 1_000_000_000


def _build_profile_cmd(base_hz: int, cfg: RadarCfg) -> str:
    p = cfg.profile_cfg
    start_freq_ghz = f"{base_hz / 1e9:.6f}"
    return (
        f"profileCfg {types.profile.PROFILE_ID_DEFAULT}"
        f" {start_freq_ghz}"
        f" {p.idle_time.to_float()}"
        f" {p.adc_start_time.to_float()}"
        f" {p.ramp_end_time.to_float()}"
        f" {p.tx_out_power.to_int()}"
        f" {p.tx_phase_shifter.to_int()}"
        f" {p.freq_slope_const.to_float()}"
        f" {p.tx_start_time.to_float()}"
        f" {p.num_adc_samples.to_int()}"
        f" {p.dig_out_sample_rate.to_int()}"
        f" {p.hpf_corner_freq1.to_int()}"
        f" {p.hpf_corner_freq2.to_int()}"
        f" {p.rx_gain.to_int()}"
    )


def _build_preamble(cfg: RadarCfg) -> list[str]:
    return [
        "flushCfg",
        f"dfeDataOutputMode {cfg.channel_cfg.dfe_output_mode.to_int()}",
        f"channelCfg {cfg.channel_cfg.rx_channel_mask.to_int()}"
        f" {cfg.channel_cfg.tx_channel_mask.to_int()}"
        f" {types.channel.SOC_CASCADING_NA}",
        f"adcCfg {types.adc.ADC_NUM_BITS_16}"
        f" {cfg.adc_cfg.adc_output_fmt.to_int()}",
        f"adcbufCfg {types.adc.SUBFRAME_IDX_LEGACY}"
        f" {types.adc.ADC_BUF_OUTPUT_FMT_COMPLEX}"
        f" {types.adc.SAMPLE_SWAP_Q_LSB_I_MSB} {types.adc.CHAN_NON_INTERLEAVED}"
        f" {types.adc.CHIRP_THRESHOLD}",
        f"lowPower {types.adc.LOW_POWER_DONT_CARE}"
        f" {cfg.adc_cfg.low_power_adc_mode.to_int()}"
    ]


def _needs_split(freq: int, base: int, count: int) -> bool:
    if _ghz_to_hz(freq) != _ghz_to_hz(base):
        return True
    if (freq - base) >= _MAX_START_FREQ_VAR_HZ:
        return True
    if count >= _MAX_CHIRPS_PER_FRAME:
        return True
    return False


def _build_frame_commands(
    base_hz: int,
    freqs: list[int],
    preamble: list[str],
    cfg: RadarCfg,
    tx_mask: int,
) -> list[str]:
    cmds = list(preamble)
    cmds.append(_build_profile_cmd(base_hz, cfg))
    for i, f_abs in enumerate(freqs):
        start_freq_var = f_abs - base_hz
        cmds.append(
            f"chirpCfg {i} {i} 0 {start_freq_var} 0 0 0 {tx_mask}"
        )
    sfcw = cfg.sfcw_cfg
    assert sfcw is not None
    frame = sfcw.frame
    cmds.append(
        f"frameCfg {types.sfcw.CHIRP_START_IDX} {len(freqs) - 1}"
        f" {frame.num_loops.to_int()} {frame.num_frames.to_int()}"
        f" {frame.frame_periodicity_ms.to_float()}"
        f" {types.sfcw.TRIGGER_SELECT_SW}"
        f" {types.sfcw.FRAME_TRIGGER_DELAY_MS}"
    )
    cmds.append(
        f"lvdsStreamCfg {types.lvds.SUBFRAME_IDX_LEGACY}"
        f" {types.lvds.ENABLE_HEADER_OFF}"
        f" {types.lvds.DATA_FMT_ADC}"
        f" {types.lvds.ENABLE_SW_OFF}"
    )
    return cmds


def build_fmcw_frames(cfg: RadarCfg,
                      logger: logging.Logger
                      ) -> list[list[str]] | None:
    fmcw = cfg.fmcw_cfg
    if fmcw is None:
        logger.error("fmcw_cfg is required for FMCW mode")
        return None
    preamble = _build_preamble(cfg)
    cmds = list(preamble)
    start_hz = int(round(fmcw.start_freq_hz.to_float()))
    cmds.append(_build_profile_cmd(start_hz, cfg))
    tx_mask = cfg.channel_cfg.tx_channel_mask.to_int()
    cmds.append(
        f"chirpCfg {types.fmcw.CHIRP_START_IDX} {types.fmcw.CHIRP_END_IDX}"
        f" {types.profile.PROFILE_ID_DEFAULT}"
        f" {types.fmcw.START_FREQ_VAR}"
        f" {types.fmcw.FREQ_SLOPE_VAR}"
        f" {types.fmcw.IDLE_TIME_VAR}"
        f" {types.fmcw.ADC_START_TIME_VAR}"
        f" {tx_mask}"
    )
    frame = fmcw.frame
    cmds.append(
        f"frameCfg {types.fmcw.CHIRP_START_IDX} {types.fmcw.CHIRP_END_IDX}"
        f" {frame.num_loops.to_int()} {frame.num_frames.to_int()}"
        f" {frame.frame_periodicity_ms.to_float()}"
        f" {types.fmcw.TRIGGER_SELECT_SW}"
        f" {types.fmcw.FRAME_TRIGGER_DELAY_MS}"
    )
    cmds.append(
        f"lvdsStreamCfg {types.lvds.SUBFRAME_IDX_LEGACY}"
        f" {types.lvds.ENABLE_HEADER_OFF}"
        f" {types.lvds.DATA_FMT_ADC}"
        f" {types.lvds.ENABLE_SW_OFF}"
    )
    logger.debug("Successfully built UART config for FMCW mode")
    return [cmds]


def build_sfcw_frames(cfg: RadarCfg,
                      logger: logging.Logger
                      ) -> list[list[str]] | None:
    sfcw = cfg.sfcw_cfg
    if sfcw is None:
        logger.error("SFCW config is required for SFCW mode")
        return None
    start = int(round(sfcw.start_freq_hz.to_float()))
    end = int(round(sfcw.end_freq_hz.to_float()))
    df = int(round(sfcw.df_hz.to_float()))
    if end <= start:
        logger.error("end_freq_hz must be > start_freq_hz")
        return None
    if df <= 0:
        logger.error("df_hz must be > 0")
        return None
    preamble = _build_preamble(cfg)
    tx_mask = cfg.channel_cfg.tx_channel_mask.to_int()

    frames: list[list[str]] = []
    current: list[int] = []
    profile_base = start
    f = start

    while f < end:
        if current and _needs_split(f, profile_base, len(current)):
            logger.debug("SFCW split: %d tones, base=%d Hz, next base=%d Hz",
                         len(current), profile_base, f)
            frames.append(
                _build_frame_commands(
                    profile_base, current, preamble,
                    cfg, tx_mask,
                )
            )
            current = []
            profile_base = f
            continue

        current.append(f)
        f += df
    if current:
        frames.append(
            _build_frame_commands(
                profile_base, current, preamble,
                cfg, tx_mask,
            )
        )
    total_tones = (end - start) // df
    logger.info("Built %d SFCW frame(s) from %d tones",
                len(frames), total_tones)
    return frames
