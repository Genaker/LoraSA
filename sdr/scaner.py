import SoapySDR
from SoapySDR import *  # SOAPY_SDR_* constants
import numpy as np
import time
import matplotlib.pyplot as plt

# Configuration parameters
start_freq = 800.1e6  # Start frequency in Hz (e.g., 900.1 MHz for S1G radio)
stop_freq = 950e6  # Stop frequency in Hz (e.g., 2.49 GHz for HiF radio)
step_size = 2e6  # Step size in Hz to match the SDR's adjusted bandwidth
sample_rate_hif = 4e6  # Adjusted sample rate in Hz for HiF
sample_rate_s1g = 2e6  # Adjusted sample rate in Hz for S1G
gain = 69  # Gain in dB
freq_resolution = 10e3  # Desired frequency resolution in Hz

# Calculate the minimum number of samples needed
min_samples_hif = int(sample_rate_hif / freq_resolution)
min_samples_s1g = int(sample_rate_s1g / freq_resolution)
duration_hif = min_samples_hif / sample_rate_hif  # Adjust duration based on minimum samples
duration_s1g = min_samples_s1g / sample_rate_s1g

# Frequency slice size for extraction (0.25 MHz)
slice_size = 0.25e6

def calculate_rssi(signal, num_of_samples):
    if num_of_samples == 0:
        return -120.0  # Default low value when no samples are available

    sum_of_squares = np.sum(np.abs(signal) ** 2)
    mean_of_squares = sum_of_squares / num_of_samples
    return 10 * np.log10(mean_of_squares + 1e-10)  # Convert to dB with safety for log(0)

def frequency_sweep():
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
    ##sdr.writeSetting(SOAPY_SDR_RX, "AGC", False)  # Disable AGC

    # Allocate a buffer for received samples
    buffer_size_hif = min_samples_hif
    rx_buffer_hif = np.zeros(buffer_size_hif, dtype=np.complex64)

    print(f"Starting infinite frequency sweep from {start_freq / 1e6} MHz to {stop_freq / 1e6} MHz...")
    print(f"Frequency resolution: {freq_resolution / 1e3} kHz")

    freq_list = []
    rssi_list = []

    # Setup plot
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], marker='o', linestyle='-', color='b')
    ax.set_title("Frequency Sweep Results")
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("RSSI (dB)")
    ax.grid(True)

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

                    freq_list.append(current_freq / 1e6)  # Store frequency in MHz
                    rssi_list.append(rssi)  # Store RSSI value

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

        # Update the plot after the full sweep
        if freq_list and rssi_list:
            line.set_data(freq_list, rssi_list)
            ax.set_xlim(min(freq_list), max(freq_list))
            ax.set_ylim(min(rssi_list) - 5, max(rssi_list) + 5)
            fig.canvas.draw()
            fig.canvas.flush_events()
            freq_list.clear()
            rssi_list.clear()

if __name__ == "__main__":
    try:
        frequency_sweep()
    except KeyboardInterrupt:
        print("Infinite sweep terminated by user.")
