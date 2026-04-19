from dataclasses import dataclass


class NumLoops:
    MIN = 1
    MAX = 255

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "NumLoops | None":
        if not isinstance(value, int) or isinstance(value, bool):
            return None
        if value < cls.MIN or value > cls.MAX:
            return None
        return cls(value)


class NumFrames:
    MIN = 0
    MAX = 65535

    def __init__(self, value: int) -> None:
        self._value = value

    def to_int(self) -> int:
        return self._value

    @classmethod
    def from_int(cls, value: object) -> "NumFrames | None":
        if not isinstance(value, int) or isinstance(value, bool):
            return None
        if value < cls.MIN or value > cls.MAX:
            return None
        return cls(value)


class FramePeriodicityMs:
    def __init__(self, value: float) -> None:
        self._value = value

    def to_float(self) -> float:
        return self._value

    @classmethod
    def from_float(cls, value: object) -> "FramePeriodicityMs | None":
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return None
        v = float(value)
        if v <= 0:
            return None
        return cls(v)


@dataclass
class FrameCfg:
    num_loops: NumLoops
    num_frames: NumFrames
    frame_periodicity_ms: FramePeriodicityMs
