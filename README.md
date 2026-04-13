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

## Firmware Build

The custom MSS firmware is built using **Code Composer Studio (CCS)** with the **TI mmWave SDK**.

### Build Steps

1. Open CCS and import the project: **File > Import > CCS Projects**
2. Browse to `firmware/1843/` and select `studio_cli_xwr18xx.projectspec`
3. Build: **Project > Build All**
4. Output binary: `mmwave_Studio_cli_xwr18xx.bin` (flash via UniFlash)

---
Project is ongoing.

