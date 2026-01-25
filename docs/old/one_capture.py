
import time
import os
import sys
import threading

from app.src.hwctl import uart_x

dll_path = r"C:\Users\Asus\Desktop\my_radar_api\app\ti-external"
if dll_path not in sys.path:
    sys.path.append(dll_path)
os.add_dll_directory(dll_path)

try:
    import radar_api
    # import old.uart as uart
except Exception as e:
    print(f"[ERR] Import error: {e}", file=sys.stderr)
    sys.exit(2)


#before main i think we should create
# _CLASS - private one that will be contain uart, radar_api(fpga)
# and call it in main for real app and for test mock (dependency injection)


def main():
    if len(sys.argv) != 2:
        print(f"użycie: {os.path.basename(sys.argv[0])} <file_prefix>", file=sys.stderr)
        sys.exit(2)
    file_prefix = sys.argv[1]
    # here to add json_file_path
    # radar_api.dca1000_config_init()
    # radar_api.read_rfdc_card_fpga_version()

    ser = None
    try:
        ser = uart_x.connect_com(uart_x.PORT, uart_x.BAUDRATE, uart_x.TIMEOUT_S)

        # uart._send_command(ser, "flushCfg")
        uart_x.radar_config(ser, uart_x.COMMANDS)

        try:
            radar_api.dca_set_file_prefix(file_prefix)
        except AttributeError:
            print("[ERR]", file=sys.stderr)
            sys.exit(3)

        print(f"Start of record: {file_prefix}")
        radar_api.dca_trigger_record()
        uart_x._send_command(ser, "sensorStart")

        time.sleep(10)

        print("OK: 1 Frame Recorded")
        radar_api.dca_stop_record()
        sys.exit(0)
    except Exception as e:
        print(f"[ERR] {e}", file=sys.stderr)
        sys.exit(1)

    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()
