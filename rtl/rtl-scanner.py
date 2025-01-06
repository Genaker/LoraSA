import numpy as np
from rtlsdr import RtlSdr
import asyncio
import traceback
import time
import os

def printxy(x, y, text):
    """
    Print text at a specific (x, y) coordinate in the console.
    :param x: Column number (1-based)
    :param y: Row number (1-based)
    :param text: The text to print
    """
    # ANSI escape code to move the cursor to (y, x)
    print(f"\033[{y};{x}H{text}", end="", flush=True)

def bar_draw(x,y,rssi,max_height=20,symbol="#"):
    normalized = rssi / (rssi + 1e-6) # Normalize to 0-1
    height = int(normalized * max_height)
    for i in range(max_height):
        printxy(x,y-i," ")
    for i in range(height):
        printxy(x,y-i,symbol)

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
    freq_labels = [
        f"{(start_freq + step * i) / 1e6:.0f}" if i % 5 == 0 else ""
        for i in range(len(data))
    ]
    print(" " * 9 + '-' * len(data))  # Bar separator
    x_axis = " " * 9
    printed = 0
    previous_label = ""
    for i, label in enumerate(freq_labels):
        if label:
            printed = 0
            previous_label = label
            x_axis += label #.center(1)  # Center the MHz label under the bar
        else:
            if printed < (5 - len(previous_label)):
                x_axis += "|"  # No space between bars
            printed += 1
    print(x_axis)
    init=True

async def read_samples_async(sdr, fft_size):
    """
    Wrapper to make the blocking read_samples method work asynchronously.
    """
    return await asyncio.to_thread(sdr.read_samples, fft_size)

async def scan_frequency_range(start_freq, end_freq, step, sdr1, sdr2, fft_size=1024*4):
    """
    Scans a frequency range using two RTL-SDR devices and calculates RSSI for each frequency.
    """
    center_frequencies = np.arange(start_freq, end_freq + step, step)
    rssi_values1 = []
    rssi_values2 = []

    x=9
    y=20

    for freq in center_frequencies:
        sdr1.center_freq = freq
        sdr2.center_freq = freq

        sdr1.read_samples(2048)
        sdr2.read_samples(2048)

        # Read samples asynchronously from both devices
        samples1, samples2 = await asyncio.gather(
            read_samples_async(sdr1, fft_size), #sdr1.read_samples(fft_size),
            read_samples_async(sdr2, fft_size),
        )
        
        # Perform FFT and calculate RSSI
        power_spectrum1 = np.abs(np.fft.fft(samples1))**2
        power_spectrum2 = np.abs(np.fft.fft(samples2))**2
        power_db1 = 10 * np.log10(power_spectrum1 + 1e-6)  # Convert to dB
        power_db2 = 10 * np.log10(power_spectrum2 + 1e-6)  # Convert to dB
        avg_rssi1 = np.percentile(power_db1, 85)  # 90th percentile RSSI for this frequency
        avg_rssi2 = np.percentile(power_db2, 85)  # 90th percentile RSSI for this frequency
        rssi_values1.append(avg_rssi1)
            #bar_draw(x,y,avg_rssi1,20,"#")
        rssi_values2.append(avg_rssi2)
            #bar_draw(x,y+24,avg_rssi2,20,"#")
        x=x+1

    return rssi_values1, rssi_values2

async def process_samples_async(samples):
    """
    Wrapper to make the blocking read_samples method work asynchronously.
    """
    return await asyncio.to_thread(process_samples, samples)

def process_samples(samples):
    rssi_values = []
    power_spectrum = np.abs(np.fft.fft(samples))**2
    power_db = 10 * np.log10(power_spectrum + 1e-6)  # Convert to dB
    avg_rssi = np.percentile(power_db, 90)  # 90th percentile RSSI for this frequency
    rssi_values.append(avg_rssi)
    return rssi_values

async def main():
    # Frequency range in Hz
    start_freq = 880e6  # 800 MHz
    end_freq = 960e6    # 900 MHz
    step = 1e6          # 1 MHz steps

    # Initialize two RTL-SDR devices
    sdr1 = RtlSdr(0)  # First SDR device
    sdr2 = RtlSdr(1)  # Second SDR device

    # List available devices
    devices = RtlSdr.get_device_serial_addresses()
    print("Available devices:", devices)

    # Set parameters for both devices
    for sdr in [sdr1, sdr2]:
        sdr.sample_rate = 2.048e6  # 2.048 MSPS
        sdr.gain = 40              # Adjust gain as needed

    try:
        while True:
           
            try:
                start_time = time.time()
                rssi_values1, rssi_values2 = await scan_frequency_range(start_freq, end_freq, step, sdr1, sdr2)
                end_time = time.time()

                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')

                # Scan frequency range with both devices simultaneously
                print(f"Scanning range: {start_freq / 1e6:.0f} MHz to {end_freq / 1e6:.0f} MHz")
                print(f"Scanning time: {end_time - start_time:.2f} seconds")
                print(f"Single MHz time: {((end_time - start_time)/(end_freq / 1e6 - start_freq / 1e6)):.2f} seconds")

                # Display ASCII bar chart for Device 1
                print("RSSI (dB) for Device 1:")
                ascii_bar_chart(rssi_values1, start_freq, step)

                # Display ASCII bar chart for Device 2
                print("\nRSSI (dB) for Device 2:")
                ascii_bar_chart(rssi_values2, start_freq, step)

            except Exception as e:
                print(f"Error during scanning: {e}")
                traceback.print_exc()
                # Optionally, wait before retrying
                time.sleep(0.5)


            #time.sleep(0.5)  # Refresh every 0.5 seconds

    except KeyboardInterrupt:
        print("\nStopping scan...")

    finally:
        sdr1.close()
        sdr2.close()

if __name__ == '__main__':
    asyncio.run(main())
