# Distance Measurement using mmWave Radars

This project aims to implement **distance measurement using an SFCW (Stepped-Frequency Continuous Wave)** technique on a **TI mmWave FMCW radar** platform (IWR1843).  
It demonstrates a full signal chain from radar configuration, data acquisition, and firmware modifications, to MATLAB-based DSP analysis.

## Project Overview

- **Hardware:**  
  - TI IWR1843BOOST  
  - TI DCA1000EVM (raw data capture)  
- **Software:**
  - Custom mss firmware supporting up to **512 unique chirps**
  - Host application in Python for **data acquisition and radar configuration** (based on TI mmWave driver DLL + PyBind11)
  - MATLAB scripts for **binary data parsing and DSP analysis**

## Documentation

A complete step-by-step guide (firmware build and flashing, building and running the Python application, board connection, MATLAB post-processing) is available in [`Instruction/instruction.pdf`](Instruction/instruction.pdf).

## Repository Structure

```
├── Instruction/                  Setup and user guide (PDF + LaTeX source)
├── radar_firmware/               Custom MSS firmware for the IWR1843
│   ├── 1843/                     CCS project specification
│   ├── mss/, common/             Firmware sources
│   └── precompiled_binaries/     Ready-to-flash binaries (custom firmware, TI demo)
├── radar_app/                    Python application for radar configuration and data acquisition
│   ├── src/                      Application sources, json5 configs, default config
│   │   └── libs/dca1000/         C++ PyBind11 module for DCA1000 communication
│   ├── build/                    PyInstaller spec and executable
│   └── scripts/                  Build and code-check scripts
└── radar_postprocessing/         MATLAB post-processing
    ├── core/
    │   ├── common/               Shared parsers and DSP utilities
    │   ├── fmcw/                 FMCW processing and displacement analysis
    │   ├── sfcw/                 SFCW processing and displacement analysis
    │   │   └── calibration/      SFCW calibration (IQ, amplitude, phase), clutter removal
    │   └── analysis/             FMCW vs SFCW comparison plots
    └── measurement_data/         Measurements used in the thesis (fmcw_sweep, sfcw_sweep)
```

