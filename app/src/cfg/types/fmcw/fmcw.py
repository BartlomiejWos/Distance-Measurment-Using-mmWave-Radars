from dataclasses import dataclass

from ..common.profile import PROFILE_ID_DEFAULT

PROFILE_ID = PROFILE_ID_DEFAULT
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
class FrameCfg:
    chirp_start_idx: int
    chirp_end_idx: int
    num_loops: int
    num_frames: int
    frame_periodicity_ms: float


@dataclass
class FmcwCfg:
    chirp: ChirpCfg
    frame: FrameCfg
