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

LORA_SA_PORT = "COM6"
DRONE_PORT = "COM4"


# lifted from SpectrumScan.py

def parse_line(line):
    """Parse a JSON line from the serial input."""

    line = line[line.index('SCAN_RESULT '):]  # support garbage interleaving with the string
    _, count, rest = line.split(' ', 2)
    return int(count), json.loads(rest.replace('(', '[').replace(')', ']'))

POLY = 0x1021
def crc16(s, c):
    c = c ^ 0xffff
    for ch in s:
        c = c ^ (ord(ch) << 8)
        for i in range(8):
            if c & 0x8000:
                c = ((c << 1) ^ POLY) & 0xffff
            else:
                c = (c << 1) & 0xffff

    return c ^ 0xffff

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

def get_candidates(data):
    highest_rssi = -100
    highest_freq = 100
    for item in data:
        if item[1] > highest_rssi:
            highest_rssi = item[1]
            highest_freq = item[0]
    return [highest_freq, highest_rssi]

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
        time.sleep(5)
        lora.write(f"SCAN -1 -1\n".encode("utf-8"))
        while True:
            # read json data from lora sa:
            try:
                line = lora.readline().decode('utf-8')
            except UnicodeDecodeError:
                continue

            if len(line) == 0:
                continue

            if 'WRAP ' in line:
                try:
                    _, c, rest = line.split(' ', 2)
                    checksum = int(c, 16)
                    so_far = crc16(rest, 0)
                except Exception as e:
                    continue

            if 'SCAN_RESULT ' in line:
                if checksum == -1:
                    continue

                c16 = crc16(line, so_far)
                if checksum != c16:
                    continue
                try:
                    count, data = parse_line(line)
                    data.sort()
                    candidate = get_candidates(data)
                    print(candidate)
                    osd_text = str2osd(f"{candidate[0]}: {candidate[1]}")
                    board.send_RAW_msg(INAV_KONRAD_SET_PILOT_NAME, osd_text)
                    heading = get_heading(board)
                    lora.write(f"HEADING {heading}\n".encode("utf-8"))
                except json.JSONDecodeError:
                    continue
