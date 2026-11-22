# Distance Measurement using mmWave Radars

This project implements high-precision **distance measurement with FMCW and SFCW (Stepped-Frequency Continuous Wave)** on a single **TI mmWave radar** platform (IWR1843), where SFCW is emulated in firmware on the FMCW hardware.  
It covers the full signal chain: firmware modifications, radar configuration, data acquisition, and MATLAB-based DSP analysis and calibration. The project was developed as part of a master's thesis at AGH University of Krakow.

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

