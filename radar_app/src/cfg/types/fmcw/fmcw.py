from dataclasses import dataclass

from ..common.frame import FrameCfg
from ..common.profile import StartFreqGHz

START_FREQ_VAR = 0
FREQ_SLOPE_VAR = 0
IDLE_TIME_VAR = 0
ADC_START_TIME_VAR = 0

CHIRP_START_IDX = 0
CHIRP_END_IDX = 0
TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0


@dataclass
class FmcwCfg:
    start_freq_hz: StartFreqGHz
    frame: FrameCfg
