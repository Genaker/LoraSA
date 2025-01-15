#!/usr/bin/python
import os
import sys
import serial
import time
import json

from yamspy import MSPy

## Constants

# INAV Konrad custom FW
INAV_KONRAD_MAX_NAME_LENGTH = 16
INAV_KONRAD_SET_PILOT_NAME = 0x5000

LORA_SA_PORT = "/dev/ttyS0"
DRONE_PORT = "/dev/ttyACM0"


def get_heading(board: MSPy) -> float:
    board.fast_read_analog()
    board.fast_read_attitude()
    board.fast_read_imu()

    return int(board.SENSOR_DATA["kinematics"][2]) # heading value


def str2osd(pilot_name):
    pilot_name_bytes = pilot_name.encode("utf-8")
    pilot_name_bytes = pilot_name_bytes[:INAV_KONRAD_MAX_NAME_LENGTH].ljust(
        INAV_KONRAD_MAX_NAME_LENGTH, b" "
    )
    return pilot_name_bytes


with MSPy(device=DRONE_PORT, loglevel="WARNING", baudrate=115200) as board:
    if board == 1:
        print("Connecting to the FC... FAILED!")
        sys.exit(1)
    else:
        try:
            lora = serial.Serial(LORA_SA_PORT, 115200, timeout=5)
        except:
            for _ in range(10):
                board.send_RAW_msg(
                    INAV_KONRAD_SET_PILOT_NAME, str2osd("err: lora board")
                )
                time.sleep(2)
                board.send_RAW_msg(
                    INAV_KONRAD_SET_PILOT_NAME, str2osd("restart needed")
                )
                time.sleep(2)
            sys.exit(1)
        board.send_RAW_msg(INAV_KONRAD_SET_PILOT_NAME, str2osd("! lora sa !"))
        time.sleep(20)
        while True:
            # tick every:
            time.sleep(0.5)

            # read json data from lora sa:
            line = lora.readline()
            if len(line) == 0:
                continue

            print(line)

            try:
                parsed = json.loads(line)
            except:
                time.sleep(0.5)
                continue
            finally:
                if parsed is not None:
                    low = parsed.get("low_range_freq")
                    high = parsed.get("high_range_freq")
                    rssi = parsed.get("value")
                    osd_text = str2osd(f"{low}-{high}:{rssi}")
                    board.send_RAW_msg(INAV_KONRAD_SET_PILOT_NAME, osd_text)
                    heading = get_heading(board)
                    lora.write(f"HEADING {heading}\n".encode("utf-8"))
