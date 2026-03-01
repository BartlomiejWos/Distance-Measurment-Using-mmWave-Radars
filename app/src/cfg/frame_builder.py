from __future__ import annotations

from src.cfg.radar import RadarCfg
from . import types

_MAX_CHIRPS_PER_FRAME = 512
_MAX_START_FREQ_VAR_HZ = 450_000_000
def _ghz_bucket(freq_hz: int) -> int:
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
        f"channelCfg {cfg.channel_cfg.rx_channel_mask.to_int()}",
        f" {cfg.channel_cfg.tx_channel_mask.to_int()}"
        f" {types.channel.SOC_CASCADING_NA}",
        f"adcCfg {types.adc.ADC_NUM_BITS_16}"
        f" {cfg.adc_cfg.adc_output_fmt.to_int()}",
        f"adcbufCfg {types.adc.SUBFRAME_IDX_LEGACY} {types.adc.ADC_BUF_OUTPUT_FMT_COMPLEX}"
        f" {types.adc.SAMPLE_SWAP_Q_LSB_I_MSB} {types.adc.CHAN_NON_INTERLEAVED}"
        f" {types.adc.CHIRP_THRESHOLD}",
        f"lowPower {types.adc.LOW_POWER_DONT_CARE}"
        f" {cfg.adc_cfg.low_power_adc_mode.to_int()}"
    ]


def _needs_split(freq: int, base: int, count: int) -> bool:
    if _ghz_bucket(freq) != _ghz_bucket(base):
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

    cmds.append(
        f"frameCfg {types.sfcw.CHIRP_START_IDX} {len(freqs) - 1}"
        f" {types.sfcw.NUMBER_OF_LOOPS} {types.sfcw.NUMBER_OF_FRAMES}"
        f" {types.sfcw.FRAME_PERIODICITY_MS} {types.sfcw.TRIGGER_SELECT_SW}"
        f" {types.sfcw.FRAME_TRIGGER_DELAY_MS}"
    )
    cmds.append(
        f"lvdsStreamCfg {types.lvds.SUBFRAME_IDX_LEGACY}"
        f" {types.lvds.ENABLE_HEADER_OFF}"
        f" {types.lvds.DATA_FMT_ADC}"
        f" {types.lvds.ENABLE_SW_OFF}"
    )
    return cmds


def build_frames(cfg: RadarCfg) -> list[list[str]]:
    sfcw = cfg.sfcw_cfg
    start = int(round(sfcw.start_freq_hz))
    end = int(round(sfcw.end_freq_hz))
    df = int(round(sfcw.df_hz))

    if end <= start:
        raise ValueError("end_freq_hz must be > start_freq_hz")
    if df <= 0:
        raise ValueError("df_hz must be > 0")

    preamble = _build_preamble(cfg)
    tx_mask = cfg.channel_cfg.tx_channel_mask.to_int()

    frames: list[list[str]] = []
    current: list[int] = []
    profile_base = start
    f = start

    while f < end:
        if current and _needs_split(f, profile_base, len(current)):
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

    return frames
