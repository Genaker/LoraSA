import argparse
import serial
import serial.tools.list_ports
import time
import re
import curses
import signal
import sys
import os
from collections import defaultdict

# Import shared utilities
try:
    from serial_utils import parse_scan_result_regex, DEFAULT_BAUDRATE, DEFAULT_TIMEOUT
except ImportError:
    print("Warning: serial_utils module not found, using local definitions", file=sys.stderr)
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 1
    
    def parse_scan_result_regex(scan_result):
        """Parse SCAN_RESULT data using regex."""
        if not scan_result or 'SCAN_RESULT' not in scan_result:
            raise ValueError("Invalid scan result format")
        pattern = r"\((\d+),\s*(-\d+)\)"
        matches = re.findall(pattern, scan_result)
        if not matches:
            raise ValueError("No valid frequency/RSSI pairs found")
        return [(int(freq), int(rssi)) for freq, rssi in matches]

# Configuration constants
DEFAULT_RESOLUTION = 1
DEFAULT_THRESHOLD = -120
DEFAULT_DB_PER_HASH = 10


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def select_serial_port():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found. Please connect a device and try again.")
        return None

    print("Available Serial Ports:")
    for i, port in enumerate(ports):
        print(f"{i + 1}: {port}")

    while True:
        try:
            choice = int(input("Select a serial port (number): "))
            if 1 <= choice <= len(ports):
                return ports[choice - 1]
            else:
                print(f"Please select a valid option between 1 and {len(ports)}.")
        except ValueError:
            print("Invalid input. Please enter a number.")


def parse_scan_result(scan_result):
    """
    Parse the SCAN_RESULT data from the serial output using regex.
    This is a wrapper around the shared utility function.
    
    :param scan_result: Raw SCAN_RESULT string.
    :return: List of tuples with (frequency in kHz, RSSI in dB).
    :raises ValueError: If parsing fails or data is invalid.
    """
    return parse_scan_result_regex(scan_result)


def group_by_frequency(data, resolution_mhz):
    """
    Group data by the specified frequency resolution.
    :param data: List of tuples (frequency in kHz, RSSI in dB).
    :param resolution_mhz: Frequency resolution in MHz for grouping.
    :return: Grouped data as a list of (group frequency in MHz, max RSSI).
    """
    grouped = defaultdict(lambda: float('-inf'))  # Default to lowest RSSI
    resolution_khz = resolution_mhz * 1000  # Convert MHz to kHz

    for freq, rssi in data:
        group_freq = (freq // resolution_khz) * resolution_khz
        grouped[group_freq] = max(grouped[group_freq], rssi)

    return sorted((freq // 1000, rssi) for freq, rssi in grouped.items())


def calculate_bar_length(rssi, db_threshold, db_per_hash, max_bar_length=15):
    """
    Calculate the bar length for the histogram based on the RSSI value and threshold.
    :param rssi: The RSSI value (dB).
    :param db_threshold: The threshold for noise filtering.
    :param db_per_hash: Number of dB per # in the bar.
    :param max_bar_length: The maximum bar length in characters.
    :return: Length of the bar as an integer.
    """
    if rssi < db_threshold:
        return 0
    effective_rssi = rssi - db_threshold  # Start bars at the threshold
    return min(max_bar_length, effective_rssi // db_per_hash)


def clear_histogram_area(stdscr, start_row, height, width):
    """
    Clears the histogram area by overwriting it with spaces.
    :param stdscr: The curses screen object.
    :param start_row: The starting row for the histogram area.
    :param height: The height of the area to clear.
    :param width: The width of the terminal.
    """
    for row in range(start_row, start_row + height):
        stdscr.addstr(row, 0, " " * width)


def initialize_colors():
    """
    Initialize color pairs for curses.
    """
    curses.start_color()
    curses.use_default_colors()

    # Define color pairs
    curses.init_pair(5, 240, -1)   # Low RSSI
    curses.init_pair(1, curses.COLOR_BLUE, -1)   # Low RSSI
    curses.init_pair(2, curses.COLOR_GREEN, -1)  # Moderate RSSI
    curses.init_pair(3, curses.COLOR_YELLOW, -1)  # High RSSI
    curses.init_pair(4, curses.COLOR_RED, -1)    # Very High RSSI


def get_color_for_rssi(rssi, use_color=True, db_threshold=0):
    """
    Determine the color based on RSSI value.
    :param rssi: The RSSI value in dB.
    :param use_color: Whether to use colored output.
    :param db_threshold: The threshold value for noise filtering.
    :return: Color pair number.
    """
    if rssi < -90 or use_color == False:
        return curses.color_pair(5)  # Blue for low RSSI
    if rssi < -80:
        return curses.color_pair(1)  # Blue for low RSSI
    elif rssi < -75:
        return curses.color_pair(2)  # Green for moderate RSSI
    elif rssi < -70:
        return curses.color_pair(3)  # Yellow for high RSSI
    else:
        return curses.color_pair(4)  # Red for very high RSSI


def draw_histogram(stdscr, data, histogram_start_row, db_threshold, db_per_hash, use_color=True, show_debug=True):
    """
    Draw the histogram with colored bars, dB labels on the left, and a two-line frequency legend.
    Optionally display debug information.
    :param stdscr: The curses screen object.
    :param data: List of tuples containing (frequency in MHz, RSSI in dB).
    :param histogram_start_row: The starting row for the histogram area.
    :param db_threshold: The threshold for noise filtering.
    :param db_per_hash: Number of dB per # in the bar.
    :param use_color: Whether to use colored output.
    :param show_debug: Whether to show debugging information.
    """
    max_height, max_width = stdscr.getmaxyx()

    histogram_height = 20
    # Clear the histogram area
    clear_histogram_area(stdscr, histogram_start_row, histogram_height, max_width)

    baseline_row = histogram_start_row + histogram_height  # Set the baseline for bars
    legend_start = baseline_row + 1          # Legends go below the baseline

    # Calculate maximum bar height
    max_bar_height = baseline_row - histogram_start_row - 1

    # Draw dB scale on the left, moved 1 row down
    for y in range(max_bar_height + 1):
        db_value = db_threshold + (max_bar_height - y) * db_per_hash
        try:
            # Offset dB scale by 1 row
            stdscr.addstr(histogram_start_row + y + 1, 0, f"{db_value:>4}")
        except curses.error:
            pass

    # Draw histogram
    for i, (freq_mhz, rssi) in enumerate(data):
        bar_x = i * 2 + 5  # Offset to align bars close to the dB legend

        if rssi >= db_threshold:
            bar_length = calculate_bar_length(rssi, db_threshold, db_per_hash)
            bar_color = get_color_for_rssi(rssi, use_color)
            # Draw the bar vertically, starting from the baseline
            for y in range(bar_length):
                char = "=" if y == bar_length - 1 else "#"
                try:
                    stdscr.addstr(baseline_row - y, bar_x, char, bar_color)
                except curses.error:
                    pass
        else:
            # Show '=' at the baseline for values below the threshold
            try:
                stdscr.addstr(baseline_row, bar_x, "=", curses.color_pair(1))  # Grey for below threshold
            except curses.error:
                pass

        # Write legends below the bars
        try:
            if i % 2 == 0:
                stdscr.addstr(legend_start, bar_x, f"{freq_mhz}")
            else:
                stdscr.addstr(legend_start + 1, bar_x, f"{freq_mhz}")
        except curses.error:
            pass

    if show_debug:
        # Display debug information if enabled
        debug_start = legend_start + 3
        display_debug_output(stdscr, data, debug_start)

    stdscr.refresh()


def display_debug_output(stdscr, data, start_line):
    """
    Display debug output below the histogram with frequency and RSSI values.
    :param stdscr: The curses screen object.
    :param data: Grouped data containing frequencies and RSSI values.
    :param start_line: The starting line for the debug output.
    """
    max_height, max_width = stdscr.getmaxyx()

    # Header
    try:
        stdscr.addstr(start_line, 0, "Frequency (MHz)   Max RSSI (dB)")
        stdscr.addstr(start_line + 1, 0, "-" * max_width)
    except curses.error as e:
        # Screen too small or cursor out of bounds - log and continue
        pass

    # Format debug data compactly with fixed width: 10 values per row
    compact_data = [f"{freq_mhz:4}:{rssi:4}" for freq_mhz, rssi in data]
    rows = [compact_data[i:i + 10] for i in range(0, len(compact_data), 10)]

    # Display each row
    for row_index, row in enumerate(rows):
        if start_line + row_index + 2 < max_height:
            line = "  ".join(row)
            try:
                stdscr.addstr(start_line + row_index + 2, 0, line[:max_width])
            except curses.error:
                pass

    stdscr.refresh()


def read_serial_data(stdscr, port, baudrate, resolution_mhz, db_threshold, db_per_hash, use_color, show_debug):
    """
    Read serial data and dynamically update the histogram with optional debug output.
    :param stdscr: The curses screen object.
    :param port: The serial port.
    :param baudrate: The baud rate.
    :param resolution_mhz: Frequency resolution in MHz for grouping.
    :param db_threshold: The threshold for noise filtering.
    :param db_per_hash: Number of dB per # in the bar.
    :param use_color: Whether to use colored output.
    :param show_debug: Whether to show debugging information.
    """
    # Set up signal handler for graceful exit
    def signal_handler(sig, frame):
        curses.endwin()
        print("\nExiting gracefully...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        initialize_colors()  # Initialize colors for the histogram
        max_height, max_width = stdscr.getmaxyx()
        histogram_start_row = 3  # Move histogram up for more space below

        with serial.Serial(port, baudrate, timeout=DEFAULT_TIMEOUT) as ser:
            time.sleep(2)  # Allow serial port to stabilize

            while True:
                try:
                    response = ser.readline().decode("utf-8").strip()
                    if not response:
                        continue
                        
                    if response.startswith("SCAN_RESULT"):
                        parsed_data = parse_scan_result(response)
                        grouped_data = group_by_frequency(parsed_data, resolution_mhz)
                        draw_histogram(
                            stdscr,
                            grouped_data,
                            histogram_start_row,
                            db_threshold,
                            db_per_hash,
                            use_color,
                            show_debug
                        )
                    elif response.startswith("LORA_RSSI"):
                        try:
                            stdscr.addstr(0, 0, "                               ", curses.color_pair(1)) 
                            stdscr.addstr(0, 0, response + "dB", curses.color_pair(1))
                        except curses.error:
                            pass
                    else:
                        stdscr.refresh()
                except UnicodeDecodeError as e:
                    # Skip malformed data
                    continue
                except ValueError as e:
                    # Invalid scan result format - skip
                    continue
    except serial.SerialException as e:
        try:
            stdscr.addstr(0, 0, f"Serial error: {e}")
            stdscr.refresh()
            stdscr.getch()
        except:
            print(f"Serial error: {e}")
    except KeyboardInterrupt:
        # Handled by signal handler
        pass
    except Exception as e:
        try:
            stdscr.addstr(0, 0, f"An unexpected error occurred: {e}")
            stdscr.refresh()
            stdscr.getch()
        except:
            print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    print("Serial Communication Script: Grouped Histogram with Adjustable Threshold")
    selected_port = select_serial_port()

    parser = argparse.ArgumentParser(
        description="Serial Communication Script with Adjustable Threshold and Histogram."
    )

    # Command-line arguments
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate for the serial connection (default: 115200)")
    parser.add_argument("--resolution", type=int, default=None, help="Frequency resolution in MHz (default: 1 MHz)")
    parser.add_argument("--threshold", type=int, default=None, help="Minimum RSSI value to start bars (default: -110)")
    parser.add_argument("--db-per-hash", type=int, default=None, help="dB per '#' in the histogram (default: 5)")
    parser.add_argument("--no-color", action="store_true", help="Disable colored output")
    parser.add_argument("--no-debug", action="store_true", help="Disable debug information")
    args = parser.parse_args()

    if selected_port:
        baudrate = args.baudrate
        if args.resolution is None:
            resolution_mhz = int(input("Enter frequency resolution in MHz (default: 1 MHz): ") or 1)
        else: 
            resolution_mhz = args.resolution
        if args.threshold is None:
            db_threshold = int(input("Enter minimum RSSI value to start bars (default: -110): ") or -110)
        else: 
            db_threshold = args.threshold
        if args.db_per_hash is None:
            db_per_hash = int(input("Enter dB per #: (default: 5): ") or 5)
        else: 
            db_per_hash = args.db_per_hash
        if args.no_debug is False:
            show_debug = input("Show debug information? (yes/no, default: yes): ").strip().lower() not in ["no", "n"]
        else: 
            show_debug = not args.no_debug
        if args.no_color is False:
            use_color = input("Use colored output? (yes/no, default: yes): ").strip().lower() not in ["no", "n"]
        else: 
            use_color = not args.no_color
        curses.wrapper(read_serial_data, selected_port, baudrate, resolution_mhz, db_threshold, db_per_hash, use_color, show_debug)

