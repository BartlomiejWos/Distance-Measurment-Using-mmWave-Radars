from dataclasses import dataclass

PROFILE_ID_DEFAULT = 0

class StartFreqGHz:
    _FREQ_LIMIT_LOW_GHZ = 76.0
    _FREQ_LIMIT_HIGH_GHZ = 81.0
    _FREQ_EXCLUDED_GHZ = {78.0}
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: float) -> "StartFreqGHz | None":
        if not isinstance(value, (int, float)):
            return None
        v = float(value)
        if v < cls._FREQ_LIMIT_LOW_GHZ or v > cls._FREQ_LIMIT_HIGH_GHZ:
            return None
        if v in cls._FREQ_EXCLUDED_GHZ:
            return None
        return cls(v)


class IdleTimeUsec:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: float) -> "IdleTimeUsec | None":
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
    def from_float(cls, value: float) -> "AdcStartTimeUsec | None":
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
    def from_float(cls, value: float) -> "RampEndTimeUsec | None":
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
    def from_int(cls, value: int) -> "TxPhaseShifter | None":
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
    def from_float(cls, value: float) -> "FreqSlopeConst | None":
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
    def from_int(cls, value: int) -> "NumAdcSamples | None":
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
    def from_int(cls, value: int) -> "HpfCornerFreq1 | None":
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
    def from_int(cls, value: int) -> "HpfCornerFreq2 | None":
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
    def from_int(cls, value: int) -> "DigOutSampleRate | None":
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
    def from_float(cls, value: float) -> "TxStartTimeUsec | None":
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
    def from_int(cls, value: int) -> "TxOutPower | None":
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
    def from_int(cls, value: int) -> "RxGain | None":
        if not isinstance(value, int):
            return None
        if value < 0:
            return None
        return cls(value)


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
