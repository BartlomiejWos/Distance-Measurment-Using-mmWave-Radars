from dataclasses import dataclass

from .types.channel import ChannelCfg
from .types.adc import ADCCfg
from .types.profile import ProfileCfg
from .types.sfcw import SfcwCfg


@dataclass
class RadarCfg:
    output_path: str
    channel_cfg: ChannelCfg
    adc_cfg: ADCCfg
    profile_cfg: ProfileCfg
    sfcw_cfg: SfcwCfg
