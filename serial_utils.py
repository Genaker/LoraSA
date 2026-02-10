#!/usr/bin/env python3
"""
Shared utilities for serial communication and data parsing.
Used by ASCII_SA.py, SpectrumScan.py, and scripts/rpi-proxy-fc.py
"""

import json
import re

# Constants
POLY = 0x1021  # CRC16 CCITT-FALSE polynomial
MAX_SCAN_COUNT = 10000  # Maximum number of scans per result
MIN_FREQUENCY = 100000  # 100 MHz minimum (in kHz)
MAX_FREQUENCY = 6000000  # 6 GHz maximum (in kHz)
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1


def crc16(s, c):
    """Calculate CRC16 CCITT-FALSE checksum.
    
    Args:
        s: String to calculate checksum for
        c: Initial CRC value
        
    Returns:
        16-bit CRC checksum
    """
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
    """Parse a SCAN_RESULT line from the serial input.
    
    Args:
        line: String containing SCAN_RESULT data
        
    Returns:
        Tuple of (count, data) where data is list of tuples [(freq, rssi), ...]
        
    Raises:
        ValueError: If line format is invalid or data doesn't pass validation
        json.JSONDecodeError: If JSON parsing fails
    """
    if not line or 'SCAN_RESULT ' not in line:
        raise ValueError("Line does not contain SCAN_RESULT")
    
    # Extract SCAN_RESULT portion, supporting garbage interleaving
    line = line[line.index('SCAN_RESULT '):]
    parts = line.split(' ', 2)
    
    if len(parts) < 3:
        raise ValueError(f"Invalid SCAN_RESULT format: expected 3 parts, got {len(parts)}")
    
    _, count_str, rest = parts
    count = int(count_str)
    
    # Validate count is reasonable
    if not (0 < count <= MAX_SCAN_COUNT):
        raise ValueError(f"Invalid count value: {count} (must be 1-{MAX_SCAN_COUNT})")
    
    # Parse JSON with replacements for tuple notation
    data = json.loads(rest.replace('(', '[').replace(')', ']'))
    
    # Validate data matches count
    if len(data) != count:
        raise ValueError(f"Data length {len(data)} does not match count {count}")
    
    # Validate frequency/RSSI pairs
    for i, item in enumerate(data):
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError(f"Invalid data item at index {i}: expected [freq, rssi] pair")
        freq, rssi = item
        if not (MIN_FREQUENCY <= freq <= MAX_FREQUENCY):
            raise ValueError(f"Invalid frequency at index {i}: {freq} (must be {MIN_FREQUENCY}-{MAX_FREQUENCY} kHz)")
        if not (-200 <= rssi <= 0):
            raise ValueError(f"Invalid RSSI at index {i}: {rssi} (must be -200 to 0 dBm)")
    
    return count, data


def parse_scan_result_regex(scan_result):
    """
    Parse SCAN_RESULT data using regex (alternative method for ASCII_SA.py).
    
    Args:
        scan_result: Raw SCAN_RESULT string.
        
    Returns:
        List of tuples with (frequency in kHz, RSSI in dB).
        
    Raises:
        ValueError: If parsing fails or data is invalid.
    """
    if not scan_result or 'SCAN_RESULT' not in scan_result:
        raise ValueError("Invalid scan result format")
    
    pattern = r"\((\d+),\s*(-\d+)\)"
    matches = re.findall(pattern, scan_result)
    
    if not matches:
        raise ValueError("No valid frequency/RSSI pairs found in scan result")
    
    return [(int(freq), int(rssi)) for freq, rssi in matches]


def validate_serial_config(port, baudrate, timeout):
    """Validate serial port configuration parameters.
    
    Args:
        port: Serial port path
        baudrate: Communication baudrate
        timeout: Read timeout in seconds
        
    Raises:
        ValueError: If any parameter is invalid
    """
    if not port or not isinstance(port, str):
        raise ValueError("Port must be a non-empty string")
    
    valid_baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    if baudrate not in valid_baudrates:
        raise ValueError(f"Baudrate {baudrate} not in valid list: {valid_baudrates}")
    
    if timeout < 0 or timeout > 300:
        raise ValueError(f"Timeout {timeout} must be between 0 and 300 seconds")
