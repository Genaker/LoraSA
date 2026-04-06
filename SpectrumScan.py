#!/usr/bin/python3
# -*- encoding: utf-8 -*-

import argparse
import serial
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import json
from datetime import datetime
from argparse import RawTextHelpFormatter

# Import shared utilities
try:
    from serial_utils import crc16, parse_scan_result, DEFAULT_BAUDRATE
except ImportError:
    # Fallback if serial_utils is not available (backward compatibility)
    print("Warning: serial_utils module not found, using local definitions", file=sys.stderr)
    DEFAULT_BAUDRATE = 115200
    
    # Local definitions as fallback
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
    
    def parse_scan_result(line):
        """Parse a JSON line from the serial input."""
        if not line or 'SCAN_RESULT ' not in line:
            raise ValueError("Line does not contain SCAN_RESULT")
        line = line[line.index('SCAN_RESULT '):]
        parts = line.split(' ', 2)
        if len(parts) < 3:
            raise ValueError(f"Invalid SCAN_RESULT format")
        _, count_str, rest = parts
        count = int(count_str)
        data = json.loads(rest.replace('(', '[').replace(')', ']'))
        return count, data

# Constants
SCAN_WIDTH = 33  # number of samples in each scanline
OUT_PATH = "out"  # output path for saved files

# Default settings
DEFAULT_COLOR_MAP = 'viridis'
DEFAULT_SCAN_LEN = 200
DEFAULT_RSSI_OFFSET = -11  # dBm offset for first power bin

def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=50, fill='█', print_end="\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        print_end    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=print_end)
    if iteration == total: 
        print()


def main():
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter, description='''\
        Parse serial data from LOG_DATA_JSON functionality.

        1. #define LOG_DATA_JSON true - add this line in main.cpp, upload to device
        2. Run the script with appropriate arguments.
        3. Once the scan is complete, output files will be saved to out/
    ''')
    parser.add_argument('port', type=str, help='COM port to connect to the device')
    parser.add_argument('--speed', default=DEFAULT_BAUDRATE, type=int,
                        help=f'COM port baudrate (defaults to {DEFAULT_BAUDRATE})')
    parser.add_argument('--map', default=DEFAULT_COLOR_MAP, type=str,
                        help=f'Matplotlib color map to use for the output (defaults to "{DEFAULT_COLOR_MAP}")')
    parser.add_argument('--len', default=DEFAULT_SCAN_LEN, type=int,
                        help=f'Number of scanlines to record (defaults to {DEFAULT_SCAN_LEN})')
    parser.add_argument('--offset', default=DEFAULT_RSSI_OFFSET, type=int,
                        help=f'Default RSSI offset in dBm (defaults to {DEFAULT_RSSI_OFFSET})')
    parser.add_argument('--buckets', default=-1, type=int,
                        help='Default number of buckets to group frequencies into; if < 1, will autodetect')

    args = parser.parse_args()

    # Create the result array
    scan_len = args.len
    arr = None

    # Scanline counter
    row = 0

    # List of frequencies
    freq_list = []

    checksum = -1
    so_far = 0

    # Open the COM port
    with serial.Serial(args.port, args.speed, timeout=None) as com:

        com.write(bytes('SCAN -1 -1\n', 'ascii'))

        lines = 0
        errors = 0
        while row < scan_len:
            # Update the progress bar
            print_progress_bar(row, scan_len)

            # Read a single line
            try:
                line = com.readline().decode('utf-8')
            except UnicodeDecodeError:
                errors += 1
                continue

            if 'WRAP ' in line:
                try:
                    _, c, rest = line.split(' ', 2)
                    checksum = int(c, 16)
                    so_far = crc16(rest, 0)
                except Exception as e:
                    errors += 1
                continue

            if 'SCAN_RESULT ' in line:
                if checksum == -1:
                    errors += 1
                    continue

                c16 = crc16(line, so_far)
                if checksum != c16:
                    errors += 1
                    checksum = -1
                    continue

                checksum = -1

                lines += 1
                try:
                    count, data = parse_scan_result(line)
                    data.sort()
                except (ValueError, json.JSONDecodeError) as e:
                    errors += 1
                    continue

                r = list(zip(*data))
                if len(r) != 2 or len(data) != count:
                    errors += 1
                    continue

                freqs, rssis = r

                if arr is None:
                    w = count if args.buckets < 1 else args.buckets
                    arr = np.zeros((scan_len, w))
                    freq_list = freqs

                for col in range(len(rssis)):
                    arr[row][col] = rssis[col]

                # Increment the row counter
                row += 1

        # tell it to stop producing SCAN_RESULTS
        com.write(bytes('SCAN 0 -1\n', 'ascii'))

    print("Read %d lines, encountered %d errors. Success rate: %.2f" %
          (lines, errors, (100 - 100 * errors / lines) if lines > 0 else 0))
    arr[arr == 0] = arr.min() - 20

    # Create the figure
    fig, ax = plt.subplots(figsize=(12, 8))

    # Display the result as heatmap
    extent = [0, scan_len, freq_list[0], freq_list[-1]]
    im = ax.imshow(arr.T, cmap=args.map, extent=extent, aspect='auto', origin='lower')
    fig.colorbar(im, label='RSSI (dBm)')

    # Set plot properties and show
    timestamp = datetime.now().strftime('%y-%m-%d %H-%M-%S')
    title = f'LoraSA Spectral Scan {timestamp}'
    plt.xlabel("Time (sample)")
    plt.ylabel("Frequency (MHz)")
    fig.suptitle(title)
    fig.canvas.manager.set_window_title(title)
    plt.savefig(f'{OUT_PATH}/{title.replace(" ", "_")}.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    main()
