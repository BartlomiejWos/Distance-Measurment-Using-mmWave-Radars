from dataclasses import dataclass

from ..common.frame import FrameCfg

START_FREQ_VAR = 0
FREQ_SLOPE_VAR = 0
IDLE_TIME_VAR = 0
ADC_START_TIME_VAR = 0

TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0


@dataclass
class ChirpCfg:
    start_idx: int
    end_idx: int


@dataclass
class FmcwCfg:
    start_freq_hz: float
    chirp_start_idx: int
    chirp_end_idx: int
    chirp: ChirpCfg
    frame: FrameCfg
