from dataclasses import dataclass


SOC_CASCADING_NA = 0


class DFEOutputMode:
    _FRAME_BASED_CHIRPS = 1
    _ADV_FRAME_CFG = 3

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "DFEOutputMode | None":
        if not isinstance(value, int):
            return None
        if value not in (cls._FRAME_BASED_CHIRPS, cls._ADV_FRAME_CFG):
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
    def from_int(cls, value: object) -> "RxChannelMask | None":
        if not isinstance(value, int):
            return None
        if value not in cls._RX:
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "RxChannelMask":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


class TxChannelMask:
    TX1 = 0b001
    _ALLOWED = {
        TX1,
    }

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "TxChannelMask | None":
        if not isinstance(value, int):
            return None
        if value not in cls._ALLOWED:
            return None
        return cls(value)

    @classmethod
    def unsafe_from_int(cls, value: int) -> "TxChannelMask":
        obj = cls.__new__(cls)
        obj._value = value
        return obj


@dataclass
class ChannelCfg:
    dfe_output_mode: DFEOutputMode
    rx_channel_mask: RxChannelMask
    tx_channel_mask: TxChannelMask
