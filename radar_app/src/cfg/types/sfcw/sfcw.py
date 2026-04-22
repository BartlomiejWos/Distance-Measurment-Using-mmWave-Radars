from dataclasses import dataclass

from ..common.frame import FrameCfg
from ..common.profile import StartFreqGHz, EndFreqGHz

CHIRP_START_IDX = 0
TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0


class DfHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "DfHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


@dataclass
class SfcwCfg:
    start_freq_ghz: StartFreqGHz
    end_freq_ghz: EndFreqGHz
    df_hz: DfHz
    frame: FrameCfg
