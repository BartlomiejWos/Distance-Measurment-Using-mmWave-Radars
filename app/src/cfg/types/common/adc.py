from dataclasses import dataclass


# ADC Buf Const Cfg
ADC_NUM_BITS_16 = 2
ADC_BUF_OUTPUT_FMT_COMPLEX = 0
SUBFRAME_IDX_LEGACY = -1
SAMPLE_SWAP_Q_LSB_I_MSB = 1
CHAN_NON_INTERLEAVED = 1
CHIRP_THRESHOLD = 1

# ADC LOW POWER RESERVED
LOW_POWER_DONT_CARE = 0


class ADCOutputFmt:
    _COMPLEX_1X = 1
    _COMPLEX_2X = 2

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "ADCOutputFmt | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._COMPLEX_1X, cls._COMPLEX_2X):
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "ADCOutputFmt":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class LowPowerAdcMode:
    _REGULAR = 0
    _LOW_POWER = 1

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "LowPowerAdcMode | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._REGULAR, cls._LOW_POWER):
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "LowPowerAdcMode":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


@dataclass
class ADCCfg:
    adc_output_fmt: ADCOutputFmt
    low_power_adc_mode: LowPowerAdcMode
