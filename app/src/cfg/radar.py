from dataclasses import dataclass

from .channel import ChannelCfg
from .adc import ADCCfg


@dataclass
class RadarCfg:
    output_path: str
    channel_cfg: ChannelCfg
    adc_cfg: ADCCfg
