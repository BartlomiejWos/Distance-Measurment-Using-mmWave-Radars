from dataclasses import dataclass

PROFILE_ID_DEFAULT = 0


class StartFreqGHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "StartFreqGHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if not _in_valid_band(v):
            return None
        return cls(v)


class IdleTimeUsec:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "IdleTimeUsec | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


class AdcStartTimeUsec:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "AdcStartTimeUsec | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


class RampEndTimeUsec:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "RampEndTimeUsec | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


class TxPhaseShifter:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "TxPhaseShifter | None":
        if not isinstance(value, int):
            return None
        if value < 0:
            return None
        return cls(value)


class FreqSlopeConst:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "FreqSlopeConst | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v < 0:
            return None
        return cls(v)


class NumAdcSamples:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "NumAdcSamples | None":
        if not isinstance(value, int):
            return None
        if value <= 0:
            return None
        if value % 2 != 0:
            return None
        return cls(value)


class HpfCornerFreq1:
    _175_KHZ = 0
    _235_KHZ = 1
    _350_KHZ = 2
    _700_KHZ = 3
    _VALID = {_175_KHZ, _235_KHZ, _350_KHZ, _700_KHZ}

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "HpfCornerFreq1 | None":
        if not isinstance(value, int):
            return None
        if value not in cls._VALID:
            return None
        return cls(value)


class HpfCornerFreq2:
    _350_KHZ = 0
    _700_KHZ = 1
    _1400_KHZ = 2
    _2800_KHZ = 3
    _VALID = {_350_KHZ, _700_KHZ, _1400_KHZ, _2800_KHZ}

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "HpfCornerFreq2 | None":
        if not isinstance(value, int):
            return None
        if value not in cls._VALID:
            return None
        return cls(value)


class DigOutSampleRate:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "DigOutSampleRate | None":
        if not isinstance(value, int):
            return None
        if value <= 0:
            return None
        return cls(value)


class TxStartTimeUsec:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "TxStartTimeUsec | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


class TxOutPower:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "TxOutPower | None":
        if not isinstance(value, int):
            return None
        if value < 0:
            return None
        return cls(value)


class RxGain:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "RxGain | None":
        if not isinstance(value, int):
            return None
        if value < 0:
            return None
        return cls(value)


_FREQ_LOW_GHZ = 77.5   # Lower bound forces VCO2 on IWR1843
_FREQ_HIGH_GHZ = 81.0


def _in_valid_band(freq_ghz: float) -> bool:
    return _FREQ_LOW_GHZ <= freq_ghz <= _FREQ_HIGH_GHZ


class EndFreqGHz:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "EndFreqGHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if not _in_valid_band(v):
            return None
        return cls(v)


@dataclass
class ProfileCfg:
    idle_time: IdleTimeUsec
    adc_start_time: AdcStartTimeUsec
    ramp_end_time: RampEndTimeUsec
    tx_out_power: TxOutPower
    tx_phase_shifter: TxPhaseShifter
    freq_slope_const: FreqSlopeConst
    num_adc_samples: NumAdcSamples
    hpf_corner_freq1: HpfCornerFreq1
    hpf_corner_freq2: HpfCornerFreq2
    dig_out_sample_rate: DigOutSampleRate
    tx_start_time: TxStartTimeUsec
    rx_gain: RxGain
