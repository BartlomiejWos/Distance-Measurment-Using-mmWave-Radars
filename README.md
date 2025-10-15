# Distance Measurement with mmWave Radars

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

---
Project is ongoing.

