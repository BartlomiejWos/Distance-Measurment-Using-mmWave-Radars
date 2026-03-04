from __future__ import annotations

from dataclasses import dataclass

from .common.channel import ChannelCfg
from .common.adc import ADCCfg
from .common.profile import ProfileCfg
from .sfcw import SfcwCfg
from .fmcw import FmcwCfg


@dataclass
class RadarCfg:
    output_path: str
    channel_cfg: ChannelCfg
    adc_cfg: ADCCfg
    profile_cfg: ProfileCfg
    sfcw_cfg: SfcwCfg
    fmcw_cfg: FmcwCfg | None = None
