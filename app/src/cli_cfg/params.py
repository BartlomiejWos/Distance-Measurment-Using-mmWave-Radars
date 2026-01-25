import logging


class DFEOutputMode:
    _FRAME_BASED_CHIRPS = 1
    _ADV_FRAME_CFG = 3

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "DFEOutputMode | None":
        if not isinstance(value, int):
            #logger.error("DFEOutputMode must be int")
            return None
        if value not in (cls._FRAME_BASED_CHIRPS, cls._ADV_FRAME_CFG):
            #logger.error(f"DFEOutputMode must be {cls._FRAME_BASED_CHIRPS} "
            #             f"or {cls._ADV_FRAME_CFG}")
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "DFEOutputMode":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class RxChannelMask:
    RX_1 = 0b0001
    RX_2 = 0b0011
    RX_3 = 0b0111
    RX_4 = 0b1111

    _RX = {
        RX_1,
        RX_2,
        RX_3,
        RX_4,
    }

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "RxChannelMask | None":
        if not isinstance(value, int):
            #logger.error("RxChannelMask must be int")
            return None
        if value not in cls._RX:
            #logger.error("RxChannelMask must be one of: 1, 3, 7, 15")
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "RxChannelMask":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class TxChannelMask:
    TX1 = 0b001
    TX2 = 0b010
    TX3 = 0b100

    AZIMUTH_2 = TX1 | TX3        # 0b101 = 5
    AZIMUTH_ELEVATION = TX1 | TX2  # 0b011 = 3
    ALL = TX1 | TX2 | TX3        # 0b111 = 7

    _ALLOWED = {
        TX1,
        TX2,
        TX3,
        AZIMUTH_2,
        AZIMUTH_ELEVATION,
        ALL,
    }

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: int) -> "TxChannelMask | None":
        if not isinstance(value, int):
            #logger.error("TxChannelMask must be int")
            return None
        if value not in cls._ALLOWED:
            #logger.error("TxChannelMask must be one of: 1, 2, 3, 4, 5, 7")
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "TxChannelMask":
        obj = cls.__new__(cls)
        obj._value = value
        return obj
