import argparse
import logging
import sys
from typing import Sequence

from app.src.hwctl.controller import HwCtl
from app.src.hwctl.radar_api_wrap import RadarApiPyd
from app.src.hwctl.uart import PySerialUart
from app.src.cli_cfg import parse
from app.src.cli_cfg import radar_cfg

def _build_logger() -> logging.Logger:
    level = logging.DEBUG
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return logging.getLogger("radar_app")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar_app",
        description="One capture using mmWave radar + DCA1000",
    )

    parser.add_argument(
        "file_prefix",
        help="File prefix for DCA1000 output",
    )


    parser.add_argument(
        "--config",
        help="Path to radar_cfg.json5",
    )
    return parser

def _prepare_commands(cli: radar_cfg.RadarCfg):
    max_freq_var = 449e6 
    N = 256 

    step = (2 * max_freq_var) / (N - 1)

    txMask = [2, 4]

    COMMANDS = [
        "flushCfg",
        f"dfeDataOutputMode {cli.channel_cfg.dfe_output_mode.to_int()}",
        "channelCfg 15 7 0",
        "adcCfg 2 1",
        "adcbufCfg -1 0 1 1 1",
        "lowPower 0 0",
        "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",
    ]

    # freq_vars = [int(round(-max_freq_var + i * step)) for i in range(N)]

    # for i, freqVar in enumerate(freq_vars):
    #     COMMANDS.append(f'chirpCfg {2*i} {2*i} 0 {freqVar} 0 0 0 {txMask[0]}')
    #     COMMANDS.append(f'chirpCfg {2*i+1} {2*i+1} 0 {freqVar} 0 0 0 {txMask[1]}')

    # COMMANDS += [
    #     f"frameCfg 0 {2*N-1} 1 1 50 1 0",
    #     "lvdsStreamCfg -1 0 1 0",
    # ]
    return COMMANDS


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    log = _build_logger()

    try:
        dll_path = r"C:\Users\Asus\Desktop\my_radar_api\app\ti-external"
        radar = RadarApiPyd(dll_path=dll_path)
        uart = PySerialUart(
            logger=log,
        )
        hw = HwCtl(uart=uart, radar=radar, logger=log)

        radar_cfg = parse.load_cfg(args.config)
        commands = _prepare_commands(radar_cfg)
        hw.one_capture(
            file_prefix=args.file_prefix,
            commands=commands
        )
        return 0

    except KeyboardInterrupt:
        log.warning("Interrupted by user")
        return 130
    except Exception:
        log.exception("Fatal error")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())