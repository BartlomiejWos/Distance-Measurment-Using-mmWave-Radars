import os
import sys

dll_path = r"C:\Users\Asus\\Desktop\my_radar_api\app\ti-external"

if dll_path not in sys.path:
    sys.path.append(dll_path)

os.add_dll_directory(dll_path)

import radar_api

radar_api.read_rfdc_card_dll_version()
radar_api.read_rfdc_card_fpga_version()