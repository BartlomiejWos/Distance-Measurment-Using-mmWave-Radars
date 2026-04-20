from dataclasses import dataclass

from ..common.frame import FrameCfg

START_FREQ_VAR = 0
FREQ_SLOPE_VAR = 0
IDLE_TIME_VAR = 0
ADC_START_TIME_VAR = 0

CHIRP_START_IDX = 0
CHIRP_END_IDX = 0
TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0

_FREQ_LOW_HZ = 76e9
_FREQ_HIGH_HZ = 81e9


def _in_valid_band(freq_hz: float) -> bool:
    return _FREQ_LOW_HZ <= freq_hz <= _FREQ_HIGH_HZ


class FmcwStartFreqHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "FmcwStartFreqHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if not _in_valid_band(v):
            return None
        return cls(v)


@dataclass
class FmcwCfg:
    start_freq_hz: FmcwStartFreqHz
    frame: FrameCfg
