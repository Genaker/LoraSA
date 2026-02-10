#!/usr/bin/python
import os
import sys
import serial
import time
import json

from yamspy import MSPy

# Import shared utilities
try:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from serial_utils import crc16, parse_scan_result as parse_line, DEFAULT_BAUDRATE
except ImportError:
    print("Warning: serial_utils module not found, using local definitions", file=sys.stderr)
    DEFAULT_BAUDRATE = 115200
    
    # Local fallback definitions
    POLY = 0x1021
    
    def crc16(s, c):
        """Calculate CRC16 CCITT-FALSE checksum."""
        c = c ^ 0xffff
        for ch in s:
            c = c ^ (ord(ch) << 8)
            for i in range(8):
                if c & 0x8000:
                    c = ((c << 1) ^ POLY) & 0xffff
                else:
                    c = (c << 1) & 0xffff
        return c ^ 0xffff
    
    def parse_line(line):
        """Parse a JSON line from the serial input."""
        if 'SCAN_RESULT ' not in line:
            raise ValueError("Line does not contain SCAN_RESULT")
        line = line[line.index('SCAN_RESULT '):]
        parts = line.split(' ', 2)
        if len(parts) < 3:
            raise ValueError(f"Invalid SCAN_RESULT format")
        _, count_str, rest = parts
        count = int(count_str)
        data = json.loads(rest.replace('(', '[').replace(')', ']'))
        return count, data

## Constants

# INAV Konrad custom FW
# https://github.com/KonradIT/inav/tree/set-pilot-name-msp
INAV_KONRAD_MAX_NAME_LENGTH = 16
INAV_KONRAD_SET_PILOT_NAME = 0x5000


# Configuration - can be overridden via environment variables
LORA_SA_PORT = os.getenv("LORA_SA_PORT", "/dev/ttyS0")
DRONE_PORT = os.getenv("DRONE_PORT", "/dev/ttyACM0")
SERIAL_BAUDRATE = int(os.getenv("SERIAL_BAUDRATE", DEFAULT_BAUDRATE))
SERIAL_TIMEOUT = int(os.getenv("SERIAL_TIMEOUT", "5"))

# For testing on Windows:
# LORA_SA_PORT = "COM6"
# DRONE_PORT = "COM4"


# use MSP to get the heading
def get_heading(board: MSPy) -> float:
    board.fast_read_analog()
    board.fast_read_attitude()
    board.fast_read_imu()

    return int(board.SENSOR_DATA["kinematics"][2]) # heading (z) value


def str2osd(pilot_name):
    pilot_name_bytes = pilot_name.encode("utf-8")
    pilot_name_bytes = pilot_name_bytes[:INAV_KONRAD_MAX_NAME_LENGTH].ljust(
        INAV_KONRAD_MAX_NAME_LENGTH, b" "
    )
    return pilot_name_bytes

# get the highest reading from the entire scan array
def get_candidates(data):
    highest_rssi = -120
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
            # Attempt to connect to the Lora ESP SA over serial
            lora = serial.Serial(LORA_SA_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
        except (serial.SerialException, OSError) as e:
            # Display error on FC OSD
            print(f"Error connecting to LoRa board: {e}")
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
        
        # On to sending data:
        board.send_RAW_msg(INAV_KONRAD_SET_PILOT_NAME, str2osd("! lora sa !"))
        time.sleep(20)

        # Tell Lora ESP SA to start sending data over serial
        lora.write(f"SCAN -1 -1\n".encode("utf-8"))
        while True:
            # read incoming WRAP and CRC data from lora sa:
            # how it looks:
            """
            SCAN_RESULT 258 [ (830000, -110), (830449, -110), (830898, -110), (831347, -110), 
            (831796, -110), (832245, -110), (832694, -110), (833143, -110), (833592, -110), 
            (834041, -110), (834490, -110), ....
            """
            try:
                line = lora.readline().decode('utf-8')
            except UnicodeDecodeError:
                continue

            if len(line) == 0:
                continue

            # Lifted from SpectrumScan.py
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
                    osd_text = str2osd(f"{candidate[0]}: {candidate[1]}")
                    board.send_RAW_msg(INAV_KONRAD_SET_PILOT_NAME, osd_text)
                    heading = get_heading(board)
                    lora.write(f"HEADING {heading}\n".encode("utf-8"))
                except json.JSONDecodeError as e:
                    print(f"Error parsing JSON: {e}")
                    continue
                except Exception as e:
                    print(f"Error processing scan result: {e}")
                    continue
