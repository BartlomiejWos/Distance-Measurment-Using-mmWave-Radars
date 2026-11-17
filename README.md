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

## Repository Structure

| Directory | Content |
|---|---|
| `radar_firmware/` | Custom MSS firmware for the IWR1843 (CCS project, sources, precompiled images) |
| `radar_app/` | Python application for radar configuration and data acquisition, including the C++ module for DCA1000 communication |
| `radar_postprocessing/` | MATLAB post-processing (FMCW, SFCW, calibration, comparison) |

## Firmware Build

The custom MSS firmware is built using **Code Composer Studio (CCS)** with the **TI mmWave SDK**.

### Build Steps

1. Open CCS and import the project: **File > Import > CCS Projects**
2. Browse to `radar_firmware/1843/` and select `studio_cli_xwr18xx.projectspec`
3. Build: **Project > Build All**
4. Output binary: `mmwave_Studio_cli_xwr18xx.bin` (flash via UniFlash)

## Documentation

A complete step-by-step guide (firmware build and flashing, building and running the Python application, MATLAB post-processing) is available in [`docs/build_and_run_instruction.pdf`](docs/build_and_run_instruction.pdf).

