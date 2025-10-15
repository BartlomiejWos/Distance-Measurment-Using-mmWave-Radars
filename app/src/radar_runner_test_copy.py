import serial
import time
import os
import sys
import threading


dll_path = r"C:\Users\Asus\Desktop\my_radar_api\app\ti-external"

if dll_path not in sys.path:
    sys.path.append(dll_path)

os.add_dll_directory(dll_path)

import radar_api

def on_event(cmd, status):
    print(f"[PY] event: cmd={cmd}, status={status}")


radar_api.dca1000_config_init()
radar_api.read_rfdc_card_fpga_version()

port = "COM22"
baudrate = 921600
timeout_sec = 3

try:
    ser = serial.Serial(port, baudrate, timeout=timeout_sec)
except serial.SerialException as e:
    sys.exit(1)

# commands = [
#     "dfeDataOutputMode 1",
#     "channelCfg 15 7 0",
#     "adcCfg 2 1",
#     "adcbufCfg -1 0 1 1 1",
#     "profileCfg 0 77 7 3 39 0 0 100 1 256 7200 0 0 30",
#     "chirpCfg 0 0 0 0 0 0 0 1",
#     "chirpCfg 1 1 0 0 0 0 0 4",
#     "frameCfg 0 1 32 100 100 1 0",
#     "lowPower 0 0",
#     "lvdsStreamCfg -1 0 1 0",
#     "sensorStart"
    # "calibMonCfg 1 1",
    # "monCalibReportCfg 1 1 0",
    # "txPowerMonCfg 1 0 0",
    # "txPowerMonCfg 1 1 0",  # może zwrócić błąd
    # "txPowerMonCfg 1 2 0",
    # "txBallbreakMonCfg 1 0",
    # "txBallbreakMonCfg 1 1",
    # "txBallbreakMonCfg 1 2",
    # "rxGainPhaseMonCfg 1 0",
    # "tempMonCfg 1 20",
    # "synthFreqMonCfg 1 0",
    # "pllConVoltMonCfg 1",
    # "dualClkCompMonCfg 1",
    # "rxIfStageMonCfg 1 0",
    # "extAnaSigMonCfg 0",
    # "pmClkSigMonCfg 1 0",
    # "rxIntAnaSigMonCfg 1 0",
    # "gpadcSigMonCfg 1"
# ]

# [profileId] [startFreq] [idleTime] [adcStartTime] [rampEndTime] [txOutPower] [txPhaseShifter] [slope] [txStartTime] [numAdcSamples] [digOutSampleRate] [hpf1CornerFreq] 
commands = [
    "dfeDataOutputMode 1",
    "channelCfg 15 7 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",  # slope = 0
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 32 100 100 1 0",
    "lowPower 0 0",
    "lvdsStreamCfg -1 0 1 0",
    "sensorStart"
]

profile_cfg = {
    "profileId": 0,
    "startFreq": 77,
    "idleTime": 7,
    "adcStartTime": 3,
    "rampEndTime": 39,
    "txOutPower": 0,
    "txPhaseShifter": 0,
    "freqSlopeConst": 0,
    "txStartTime": 1,
    "numAdcSamples": 256,
    "digOutSampleRate": 7200,
    "hpfCornerFreq1": 0,
    "hpfCornerFreq2": 0,
    "rxGain": 30
}


def send_command(cmd):
    ser.write((cmd + '\n').encode('utf-8'))
    print(f">>> sended {cmd}\n")
    try:
        for _ in range(3):
            resp = ser.readline().decode('utf-8').strip()
            # if len(resp) == 0:
            #     break
            print(f"<<< {resp}")
            # if "Done" in resp or ">" in resp:
            #     break
    except Exception as e:
        print(f"Error: {e}")
        return False
    return True


for cmd in commands:
    time.sleep(0.2)
    response = send_command(cmd)

def trigger_record():
    radar_api.dca_trigger_record()


# record_thread = threading.Thread(target=trigger_record)
# record_thread.start()

radar_api.dca_trigger_record()
time.sleep(1)

# send_command("sensorStart")


record_duration = 20

time.sleep(record_duration)

radar_api.dca_stop_record()

ser.close()






