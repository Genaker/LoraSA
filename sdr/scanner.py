import sys

import SoapySDR
from SoapySDR import *  # SOAPY_SDR_* constants

import numpy as np
import time
import signal

# set log level to error
SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_ERROR)

# Configuration parameters
start_freq = 800.1e6  # Start frequency in Hz (e.g., 900.1 MHz for S1G radio)
stop_freq = 950e6  # Stop frequency in Hz (e.g., 2.49 GHz for HiF radio)
step_size_default = 0.25e6  # Default step size in Hz
sample_rate_hif = 4e6  # Adjusted sample rate in Hz for HiF
gain = 69  # Gain in dB
freq_resolution = 10e3  # Desired frequency resolution in Hz

# Calculate the minimum number of samples needed
min_samples_hif = int(sample_rate_hif / freq_resolution)
duration_hif = min_samples_hif / sample_rate_hif  # Adjust duration based on minimum samples

# Frequency slice size for extraction (0.25 MHz)
slice_size = 0.25e6

def calculate_rssi(signal, num_of_samples):
    if num_of_samples == 0:
        return -120.0  # Default low value when no samples are available

    sum_of_squares = np.sum(np.abs(signal) ** 2)
    mean_of_squares = sum_of_squares / num_of_samples
    return 10 * np.log10(mean_of_squares + 1e-10)  # Convert to dB with safety for log(0)

def render_ascii_chart(data):
    if not data:
        print("No data to render.")
        return

    max_rssi = max(data.values())
    min_rssi = min(data.values())
    chart_width = 50  # Width of the chart in characters

    print("\nFrequency Sweep Results (ASCII Chart):")
    for freq, rssi in data.items():
        # Scale RSSI values to fit within chart width
        scaled_rssi = int(((rssi - min_rssi) / (max_rssi - min_rssi)) * chart_width) if max_rssi != min_rssi else chart_width // 2
        bar = "#" * scaled_rssi
        print(f"{freq:.3f} MHz | {rssi:.2f} dB | {bar}")

def render_graphical_chart(data):
    import matplotlib.pyplot as plt
    if not data:
        print("No data to render.")
        return

    freqs = list(data.keys())
    rssis = list(data.values())

    plt.figure(figsize=(10, 6))
    plt.plot(freqs, rssis, marker="o", linestyle="-", color="b")
    plt.title("Frequency Sweep Results (Graphical Chart)")
    plt.xlabel("Frequency (MHz)")
    plt.ylabel("RSSI (dB)")
    plt.grid(True)
    plt.show()

def frequency_sweep(output_type="ascii", step_size=step_size_default):
    try:
        device = "Cariboulite"
        # Create and configure the SDR device
        sdr = SoapySDR.Device({"driver": device, "channel": "HiF"})  # Replace 'rtlsdr' with your SDR driver
        print("SDR device initialized.")

        # Check if FPGA is already programmed
        if sdr.hasHardwareTime():
            print("FPGA is already programmed and operational.")
        else:
            print("FPGA is not programmed. Initializing...")
            # Add any necessary initialization logic here

    except Exception as e:
        print(f"Error initializing SDR device: {e}")

    # Set up HiF radio
    sdr.setSampleRate(SOAPY_SDR_RX, 0, sample_rate_hif)
    sdr.setGain(SOAPY_SDR_RX, 0, gain)

    # Allocate a buffer for received samples
    buffer_size_hif = min_samples_hif
    rx_buffer_hif = np.zeros(buffer_size_hif, dtype=np.complex64)

    print(f"Starting infinite frequency sweep from {start_freq / 1e6} MHz to {stop_freq / 1e6} MHz...")
    print(f"Frequency resolution: {freq_resolution / 1e3} kHz")

    freq_rssi_map = {}

    while True:
        current_freq = start_freq
        while current_freq <= stop_freq:
            start_time = time.time()

            print(f"Tuning to center frequency {current_freq / 1e6} MHz")
            try:
                sdr.setFrequency(SOAPY_SDR_RX, 0, current_freq)
                time.sleep(0.01)  # Add a 10 ms delay after setting the frequency
            except Exception as e:
                print(f"Error setting frequency: {e}")
                current_freq += step_size
                continue

            # Start streaming
            try:
                stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
                sdr.activateStream(stream)

                # Collect samples
                sr = sdr.readStream(stream, [rx_buffer_hif], len(rx_buffer_hif))
                if sr.ret > 0:
                    # Calculate RSSI (Received Signal Strength Indicator)
                    rssi = calculate_rssi(rx_buffer_hif, sr.ret)
                    calibration_offset = -50  # Example offset for realistic RSSI values
                    rssi += gain + calibration_offset  # Adjust RSSI for gain and calibration
                    print(f"Center Frequency: {current_freq / 1e6:.3f} MHz, RSSI: {rssi:.2f} dB")

                    freq_rssi_map[current_freq / 1e6] = rssi  # Store frequency in MHz as key and RSSI as value

                iteration_time = time.time() - start_time
                print(f"Iteration completed in {iteration_time:.3f} seconds")

            except Exception as e:
                print(f"Error during stream or processing: {e}")

            finally:
                # Stop streaming and clean up
                try:
                    sdr.deactivateStream(stream)
                    sdr.closeStream(stream)
                except Exception as e:
                    print(f"Error cleaning up stream: {e}")

            current_freq += step_size

        # Render the chart after the full sweep
        if freq_rssi_map:
            if output_type == "ascii":
                render_ascii_chart(freq_rssi_map)
            elif output_type == "graphical":
                render_graphical_chart(freq_rssi_map)
            else:
                print("Invalid output type. Choose 'ascii' or 'graphical'.")
            freq_rssi_map.clear()

if __name__ == "__main__":
    def catch_quit(sig, frame):
        sdr.deactivateStream(stream)
        sdr.closeStream(stream)
        sys.exit(0)

    signal.signal(signal.SIGINT, catch_quit)
    
    output_type = "ascii"  # Default to ASCII output
    step_size = step_size_default  # Default step size
    if len(sys.argv) > 1:
        output_type = sys.argv[1].strip().lower()
    if len(sys.argv) > 2:
        try:
            step_size = float(sys.argv[2])
        except ValueError:
            print("Invalid step size provided. Using default.")
    frequency_sweep(output_type=output_type, step_size=step_size)
