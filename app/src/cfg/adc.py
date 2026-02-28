from dataclasses import dataclass


class NumADCBits:
    _16_BITS = 2

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "NumADCBits | None":
        if not isinstance(value, int):
            return None
        if value != cls._16_BITS:
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "NumADCBits":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


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


@dataclass
class ADCCfg:
    num_adc_bits: NumADCBits
    adc_output_fmt: ADCOutputFmt
