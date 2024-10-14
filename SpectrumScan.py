#!/usr/bin/python3
# -*- encoding: utf-8 -*-

import argparse
import serial
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from datetime import datetime
from argparse import RawTextHelpFormatter

import json

# number of samples in each scanline
SCAN_WIDTH = 33

# scanline Serial start/end markers
SCAN_MARK_START = 'SCAN '
SCAN_MARK_FREQ = 'FREQ '
SCAN_MARK_END = ' END'

# output path
OUT_PATH = 'out'

# default settings
DEFAULT_BAUDRATE = 115200
DEFAULT_COLOR_MAP = 'viridis'
DEFAULT_SCAN_LEN = 200
DEFAULT_RSSI_OFFSET = -11

# Print iterations progress
# from https://stackoverflow.com/questions/3173320/text-progress-bar-in-terminal-with-block-characters
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 50, fill = '█', printEnd = "\r"):
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
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    if iteration == total: 
        print()

def parse_line(line):
    json_line = json.loads(line)
    return json_line

def main():
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter, description='''
        Parse serial data from LOG_DATA_JSON functionality.

        1. #define LOG_DATA_JSON true - add this line in main.cpp, upload to device
        2. Run the script with appropriate arguments.
        3. Once the scan is complete, output files will be saved to out/
    ''')
    parser.add_argument('port',
        type=str,
        help='COM port to connect to the device')
    parser.add_argument('--speed',
        default=DEFAULT_BAUDRATE,
        type=int,
        help=f'COM port baudrate (defaults to {DEFAULT_BAUDRATE})')
    parser.add_argument('--map',
        default=DEFAULT_COLOR_MAP,
        type=str,
        help=f'Matplotlib color map to use for the output (defaults to "{DEFAULT_COLOR_MAP}")')
    parser.add_argument('--len',
        default=DEFAULT_SCAN_LEN,
        type=int,
        help=f'Number of scanlines to record (defaults to {DEFAULT_SCAN_LEN})')
    parser.add_argument('--offset',
        default=DEFAULT_RSSI_OFFSET,
        type=int,
        help=f'Default RSSI offset in dBm (defaults to {DEFAULT_RSSI_OFFSET})')
    parser.add_argument('--freq',
        default=-1,
        type=float,
        help=f'Default starting frequency in MHz')
    args = parser.parse_args()

    # create the result array
    arr = np.zeros((scan_len, SCAN_WIDTH))

    # scanline counter
    row = 0

    # list of frequencies
    freq_list = []

    # open the COM port
    with serial.Serial(args.port, args.speed, timeout=None) as com:
        while(True):
            # update the progress bar
            printProgressBar(row, scan_len)

            # read a single line
            try:
                line = com.readline().decode('utf-8')
            except:
                continue
            print(line)
            if "{" in line:
                try:
                    data = parse_line(line)
                except:
                    continue

                freq = data["low_range_freq"]
                rssi = int(data["value"])

                if freq not in freq_list:
                    freq_list.append(freq)
                
                col = freq_list.index(freq)
                arr[row][col] = rssi
                
                # increment the row counter
                row = row + 1

                # check if we're done
                if (row >= scan_len):
                    break

    # create the figure
    fig, ax = plt.subplots(figsize=(12, 8))

    # display the result as heatmap
    extent = [0, scan_len, freq_list[0], freq_list[-1]]
    im = ax.imshow(arr.T, cmap=args.map, extent=extent, aspect='auto', origin='lower')
    fig.colorbar(im, label='RSSI (dBm)')

    # set some properties and show 
    timestamp = datetime.now().strftime('%y-%m-%d %H-%M-%S')
    title = f'RadioLib SX126x Spectral Scan {timestamp}'
    plt.xlabel("Time (sample)")
    plt.ylabel("Frequency (MHz)")
    fig.suptitle(title)
    fig.canvas.manager.set_window_title(title)
    plt.savefig(f'{OUT_PATH}/{title.replace(" ", "_")}.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    main()
