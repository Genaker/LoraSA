import numpy as np
from rtlsdr import RtlSdr
import time
import os

def ascii_bar_chart(data, start_freq, step, max_height=20, symbol='#'):
    """
    Converts an array of RSSI values into an ASCII vertical bar chart with RSSI labels on the left and MHz labels under every 5th step.
    """
    min_rssi = min(data)
    max_rssi = max(data)
    normalized = [(x - min_rssi) / (max_rssi - min_rssi + 1e-6) for x in data]  # Normalize to 0-1
    heights = [int(value * max_height) for value in normalized]
    
    # Generate left-aligned RSSI labels
    rssi_range = np.linspace(max_rssi, min_rssi, max_height)
    rssi_labels = [f"{rssi:.1f} dB".rjust(8) for rssi in rssi_range]

    # Print the vertical bars with RSSI labels
    for level, label in zip(range(max_height, 0, -1), rssi_labels):
        line = label + ' '  # Add RSSI label
        for height in heights:
            line += symbol if height >= level else ' '
        print(line)
    
    # Print x-axis
    freq_labels = [f"{(start_freq + step * i) / 1e6:.0f}" if i % 5 == 0 else "" for i in range(len(data))]
    print(" " * 8 + '-' * len(data))  # Bar separator
    x_axis = " " * 8
    printed = 0
    previous_label = ""
    for i, label in enumerate(freq_labels):
        if label:
            printed = 0
            previous_label = label
            x_axis += label #.center(1)  # Center the MHz label under the bar
        else:
            if printed < (5 - len(previous_label)):
                x_axis += " "  # No space between bars
            printed += 1
    print(x_axis)

def scan_frequency_range(start_freq, end_freq, step, sdr, fft_size=2*1024):
    """
    Scans a frequency range using an RTL-SDR device and calculates RSSI for each frequency.
    """
    center_frequencies = np.arange(start_freq, end_freq + step, step)
    rssi_values = []
    
    for freq in center_frequencies:
        sdr.center_freq = freq
        samples = sdr.read_samples(fft_size)
        
        # Perform FFT and calculate RSSI
        power_spectrum = np.abs(np.fft.fft(samples))**2
        power_db = 10 * np.log10(power_spectrum + 1e-6)  # Convert to dB
        avg_rssi = np.percentile(power_db, 90)  # 95 percentile RSSI for this frequency
        rssi_values.append(avg_rssi)
    
    return rssi_values

def main():
    # Frequency range in Hz
    start_freq = 800e6  # 800 MHz
    end_freq = 900e6   # 1200 MHz
    step = 1e6          # 1 MHz steps

    # Initialize RTL-SDR
    sdr = RtlSdr()
    sdr.sample_rate = 2.048e6  # 2.048 MSPS
    sdr.gain = 10             # Adjust gain as needed

    try:
        while True:
            start_time = time.time()
            rssi_values = scan_frequency_range(start_freq, end_freq, step, sdr)
            end_time = time.time()

            # Clear screen
            os.system('clear' if os.name == 'posix' else 'cls')

            # Scan frequency range
            print(f"Scanning range: {start_freq / 1e6:.0f} MHz to {end_freq / 1e6:.0f} MHz")
            print(f"Scanning time: {end_time - start_time:.2f} seconds")

            # Display ASCII bar chart with RSSI and MHz labels
            print("RSSI (dB):")
            ascii_bar_chart(rssi_values, start_freq, step)
            #time.sleep(0.5)  # Refresh every 0.5 seconds

    except KeyboardInterrupt:
        print("\nStopping scan...")

    finally:
        sdr.close()

if __name__ == '__main__':
    main()
