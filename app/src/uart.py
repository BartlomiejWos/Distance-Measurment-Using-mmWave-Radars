import serial
import sys
import time


PORT = "COM22"
BAUDRATE = 921600 #921600 for studio cli 115200 for mmwdemo
TIMEOUT_S= 3


def connect_com(port, baudrate, timeout_sec):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout_sec)
        print(f"Connected with {port} @ {baudrate}")
    except serial.SerialException as e:
        print(f"Error when opening COM: {e}")
        sys.exit(1)
    return ser


def _send_command(ser, cmd):
    ser.write((cmd + '\n').encode('utf-8'))
    print(f">>> sended {cmd}\n")
    try:
        for _ in range(3):
            resp = ser.readline().decode('utf-8').strip()
            # if len(resp) == 0:
            #     break
            print(f"<<< {resp}")
            if "Done" in resp:
                print(f"<<< {resp}")
                break
    except Exception as e:
        print(f"Error: {e}")
        return False
    return True


def radar_config(ser,commands):
    for cmd in commands:
        time.sleep(0.8)
        _send_command(ser, cmd)

max_freq_var = 449e6 
N = 256 

step = (2 * max_freq_var) / (N - 1)

txMask = [2, 4]

COMMANDS = [
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 7 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "lowPower 0 0",
    "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",
]

freq_vars = [int(round(-max_freq_var + i * step)) for i in range(N)]

for i, freqVar in enumerate(freq_vars):
    COMMANDS.append(f'chirpCfg {2*i} {2*i} 0 {freqVar} 0 0 0 {txMask[0]}')
    COMMANDS.append(f'chirpCfg {2*i+1} {2*i+1} 0 {freqVar} 0 0 0 {txMask[1]}')

COMMANDS += [
    f"frameCfg 0 {2*N-1} 1 1 50 1 0",
    "lvdsStreamCfg -1 0 1 0",
]


# max_freq_var = 450e6  # ±450 MHz ->  Hz
# step = 15e6           # 
# txMask = [1, 4]

# COMMANDS = [
#     "flushCfg",
#     "dfeDataOutputMode 1",
#     "channelCfg 15 7 0",
#     "adcCfg 2 1",
#     "adcbufCfg -1 0 1 1 1",
#     "lowPower 0 0",
#     "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",
# ]

# freq_vars = list(range(int(-max_freq_var), int(max_freq_var + 1), int(step)))
# N = len(freq_vars)

# for i, freqVar in enumerate(freq_vars):
#     COMMANDS.append(f'chirpCfg {2*i} {2*i} 0 {freqVar:.0f} 0 0 0 {txMask[0]}')
#     COMMANDS.append(f'chirpCfg {2*i+1} {2*i+1} 0 {freqVar:.0f} 0 0 0 {txMask[1]}')

# COMMANDS += [
#     f"frameCfg 0 {2*N-1} 1 1 50 1 0",
#     "lvdsStreamCfg -1 0 1 0",
# ]


# [profileId] [startFreq] [idleTime] [adcStartTime] [rampEndTime] [txOutPower] [txPhaseShifter] [slope] [txStartTime] [numAdcSamples] [digOutSampleRate] [hpf1CornerFreq] 
# COMMANDS = [
#     "dfeDataOutputMode 1",
#     "channelCfg 15 7 0",
#     "adcCfg 2 1",
#     "adcbufCfg -1 0 1 1 1",
#     "profileCfg 0 77 7 3 39 0 0 0 1 256 7200 0 0 30",  # slope = 0
#     "chirpCfg 0 0 0 0 0 0 0 1",
#     "chirpCfg 1 1 0 0 0 0 0 4",
#     # "frameCfg 0 1 32 100 100 1 0",
#     "frameCfg 0 1 1 1 50 1 0",
#     "lowPower 0 0",
#     "lvdsStreamCfg -1 0 1 0",
#     "sensorStart"
# ]