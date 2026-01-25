from dataclasses import dataclass

from . import params



@dataclass
class ChannelCfg:
    dfe_output_mode: params.DFEOutputMode
    rx_channel_mask: params.RxChannelMask
    tx_channel_mask: params.TxChannelMask


@dataclass
class RadarCfg:
    channel_cfg: ChannelCfg
    
