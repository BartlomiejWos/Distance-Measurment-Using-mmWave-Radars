from dataclasses import dataclass

CHIRP_START_IDX = 0
NUMBER_OF_LOOPS = 1
NUMBER_OF_FRAMES = 1
FRAME_PERIODICITY_MS = 50
TRIGGER_SELECT_SW = 1
FRAME_TRIGGER_DELAY_MS = 0


@dataclass
class SfcwCfg:
    start_freq_hz: float  # e.g. 77e9
    end_freq_hz: float    # e.g. 78e9
    df_hz: float          # e.g. 1e6
