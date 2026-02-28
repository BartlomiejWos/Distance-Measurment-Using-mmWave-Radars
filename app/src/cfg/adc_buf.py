class SubFrameIdx:
    _ALL = -1  # apply to all subframes / legacy mode

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "SubFrameIdx | None":
        if not isinstance(value, int):
            return None
        # -1 or >=0 subframe number
        if value < -1:
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "SubFrameIdx":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class ADCBufOutputFmt:
    _COMPLEX = 0
    _REAL = 1

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "ADCBufOutputFmt | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._COMPLEX, cls._REAL):
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "ADCBufOutputFmt":
        obj = cls.__new__(cls)
        obj._value = value
        return obj

    def is_supported(self) -> bool:
        return self._value == self._COMPLEX


class SampleSwap:
    _Q_LSB_I_MSB = 1  # only supported

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "SampleSwap | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._I_LSB_Q_MSB, cls._Q_LSB_I_MSB):
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "SampleSwap":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class ChanInterleave:
    _NON_INTERLEAVED = 1

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "ChanInterleave | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._INTERLEAVED, cls._NON_INTERLEAVED):
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "ChanInterleave":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class ChirpThreshold:
    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "ChirpThreshold | None":
        if not isinstance(value, int):
            return None
        if value < 0 or value > 8:
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "ChirpThreshold":
        obj = cls.__new__(cls)
        obj._value = value
        return obj
