from dataclasses import dataclass

CHIRP_START_IDX = 0
NUMBER_OF_LOOPS = 1
NUMBER_OF_FRAMES = 1
FRAME_PERIODICITY_MS = 50
TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0

_FREQ_LOW_HZ = 76e9
_FREQ_HIGH_HZ = 81e9
_FREQ_EXCLUDED_LOW_HZ = 78e9
_FREQ_EXCLUDED_HIGH_HZ = 79e9


def _in_valid_band(freq_hz: float) -> bool:
    if freq_hz < _FREQ_LOW_HZ or freq_hz > _FREQ_HIGH_HZ:
        return False
    if _FREQ_EXCLUDED_LOW_HZ <= freq_hz < _FREQ_EXCLUDED_HIGH_HZ:
        return False
    return True


class SfcwStartFreqHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "SfcwStartFreqHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if not _in_valid_band(v):
            return None
        return cls(v)


class SfcwEndFreqHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "SfcwEndFreqHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if not _in_valid_band(v):
            return None
        return cls(v)


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
    start_freq_hz: SfcwStartFreqHz
    end_freq_hz: SfcwEndFreqHz
    df_hz: DfHz
