from typing import Any

_COMMON: dict[str, Any] = {
    # When running with the robot setup, set the output path
    # and the COM port according to the PC used.
    "output_path": "radar_postprocessing\\data",
    "com_port": "COM22",
    "channel_cfg": {
        "dfe_output_mode": 1,
        "rx_channel_mask": 15,
        "tx_channel_mask": 1,
    },
    "adc_cfg": {
        "adc_output_fmt": 1,
        "low_power_adc_mode": 0,
    },
}

DEFAULT_FMCW_CFG: dict[str, Any] = {
    **_COMMON,
    "profile_cfg": {
        "idle_time": 10,
        "adc_start_time": 4,
        "ramp_end_time": 145,
        "tx_out_power": 0,
        "tx_phase_shifter": 0,
        "freq_slope_const": 7.2,
        "num_adc_samples": 1000,
        "hpf_corner_freq1": 0,
        "hpf_corner_freq2": 0,
        "dig_out_sample_rate": 7200,
        "tx_start_time": 1,
        "rx_gain": 30,
    },
    "fmcw_cfg": {
        "start_freq_Ghz": 77.0,
        "frame": {
            "num_loops": 128,
            "num_frames": 1,
            "frame_periodicity_ms": 50,
        },
    },
}

DEFAULT_SFCW_CFG: dict[str, Any] = {
    **_COMMON,
    "profile_cfg": {
        "idle_time": 10,
        "adc_start_time": 4,
        "ramp_end_time": 120,
        "tx_out_power": 0,
        "tx_phase_shifter": 0,
        "freq_slope_const": 0,
        "num_adc_samples": 256,
        "hpf_corner_freq1": 0,
        "hpf_corner_freq2": 0,
        "dig_out_sample_rate": 7200,
        "tx_start_time": 1,
        "rx_gain": 30,
    },
    "sfcw_cfg": {
        "start_freq_Ghz": 77.0,
        "end_freq_Ghz": 78.0,
        "df_hz": 1e6,
        "frame": {
            "num_loops": 1,
            "num_frames": 1,
            "frame_periodicity_ms": 150,
        },
    },
}

DEFAULT_CFG: dict[str, Any] = DEFAULT_FMCW_CFG
