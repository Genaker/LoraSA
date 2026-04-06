# Troubleshooting Guide

## Common Issues and Solutions

### Build and Compilation Issues

#### Error: "Unknown board ID"
**Symptom**: Build fails with `Error: Unknown board ID 'heltec_wifi_lora_32_V3'`

**Solution**: Update your ESP32 Espressif catalog:
```bash
pio pkg update -g -p espressif32
```

#### Build Timeout or Slow First Build
**Symptom**: First compilation takes very long (>5 minutes)

**Solution**: This is normal - PlatformIO needs to download and compile all libraries on first run. Subsequent builds will be much faster (~30-60 seconds).

---

### Upload Issues

#### Cannot Upload to Board
**Symptom**: Upload fails with "Failed to connect to ESP32"

**Solution**:
1. Press and hold **BOOT** button
2. Press **RESET** button once
3. Release **BOOT** button
4. Try upload again within 5 seconds

#### USB Driver Not Found (Windows)
**Symptom**: Device not recognized or COM port not showing

**Solution**: Install CP2101 USB drivers:
- Download from: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- For Windows: Use the executable installer
- For macOS: Use legacy driver if standard driver doesn't work

---

### Runtime Errors

#### No Display Output
**Symptom**: Device powers on but screen stays blank

**Solution**:
1. Check if correct board is selected in `platformio.ini`
2. Verify `src_dir` matches your board type:
   - `src` for OLED displays (Heltec V3, LilyGo)
   - `tft_src` for TFT displays (Vision Master T190)
   - `eink_src` for e-ink displays (Vision Master E290)

#### Scan Results Show No Signals
**Symptom**: Spectrum analyzer runs but shows no activity

**Solution**:
1. Check antenna is properly connected
2. Verify frequency range matches your region's LoRa bands
3. Adjust threshold in configuration (try lower value like -120 dBm)
4. Test with known active frequency (e.g., WiFi 2.4 GHz)

#### Python Scripts Fail to Connect
**Symptom**: `ASCII_SA.py` or `SpectrumScan.py` cannot open serial port

**Solution**:
1. Close Arduino IDE or PlatformIO serial monitor (only one program can use serial port)
2. Check permissions on Linux:
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```
3. Verify correct port:
   - Linux: Usually `/dev/ttyUSB0` or `/dev/ttyACM0`
   - Windows: Check Device Manager for COM port number
   - macOS: Usually `/dev/cu.usbserial-*`

4. Set port via environment variable:
   ```bash
   export LORA_SA_PORT=/dev/ttyUSB0
   python3 ASCII_SA.py
   ```

---

### Configuration Issues

#### OSD Feature Not Working
**Symptom**: Compiled with OSD_ENABLED but no video output

**Solution**:
1. Verify DFRobot OSD wiring (see main README)
2. Check SPI pins match your board configuration
3. Test camera independently before connecting to OSD

#### Custom Frequency Range Not Working
**Symptom**: Device scans wrong frequencies despite configuration

**Solution**:
1. Ensure `SCAN_DIAPAZONES` is properly formatted:
   ```c
   int SCAN_DIAPAZONES[] = {850890, 920950}; // Two ranges: 850-890 and 920-950
   ```
2. If using single range, set `RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN`
3. Frequency values must be within radio chip capabilities:
   - SX1262: 150-960 MHz
   - SX1280: 2400-2500 MHz
   - LR1121: 150-960 MHz, 2400-2500 MHz

---

### Python Script Errors

#### ModuleNotFoundError: No module named 'serial_utils'
**Symptom**: Python scripts fail with import error

**Solution**: Ensure you're running scripts from the repository root:
```bash
cd /path/to/LoraSA
python3 ASCII_SA.py
```

Or install in development mode:
```bash
export PYTHONPATH="${PYTHONPATH}:/path/to/LoraSA"
```

#### ValueError: Invalid SCAN_RESULT format
**Symptom**: Script crashes when parsing serial data

**Solution**:
1. This indicates corrupted serial data
2. Check USB cable quality (use high-quality, short cables)
3. Reduce baudrate in configuration if errors persist:
   ```bash
   export SERIAL_BAUDRATE=57600
   ```
4. Update to latest firmware version

---

### Performance Issues

#### Slow Scan Rate
**Symptom**: Less than 1 scan per second

**Solution**:
1. Reduce scan range: smaller `FREQ_END - FREQ_BEGIN`
2. Increase bandwidth setting (faster but less sensitive)
3. Disable waterfall if enabled
4. Disable WiFi/BT scanning if not needed

#### High Battery Drain
**Symptom**: Battery depletes quickly

**Solution**:
1. Reduce screen brightness
2. Use power-saving mode (press P button on startup)
3. Disable Bluetooth/WiFi if not in use
4. Use sleep mode between scans

---

### Network and Communication

#### BLE/WiFi Scanning Not Working
**Symptom**: `WIFI_SCANNING_ENABLED` or `BT_SCANNING_ENABLED` doesn't show results

**Solution**:
1. Ensure OSD is enabled (required for WiFi/BT scanning)
2. Verify defines are set:
   ```c
   #define OSD_ENABLED true
   #define WIFI_SCANNING_ENABLED true
   #define BT_SCANNING_ENABLED true
   ```
3. Check antenna placement (WiFi/BT uses different antenna than LoRa on some boards)

#### LoRa Communication Between Devices Fails
**Symptom**: Data not transmitting via LoRa

**Solution**:
1. Verify both devices use same frequency configuration
2. Check `is_host` setting (one device should be host, one client)
3. Ensure LoRa TX/RX are properly configured in `lib/config/config.h`
4. Test with shorter distance first (<100m)

---

## Getting Additional Help

If none of these solutions work:

1. **Check Serial Monitor Output**: Connect via USB and monitor serial output for error messages
   ```bash
   pio device monitor
   ```

2. **Enable Debug Logging**: Add to build flags:
   ```ini
   build_flags = -DCORE_DEBUG_LEVEL=5
   ```

3. **Report Issue**: Include:
   - Board model and version
   - PlatformIO environment used
   - Complete error message
   - Serial output log
   - Steps to reproduce

4. **Community Resources**:
   - GitHub Issues: https://github.com/Genaker/LoraSA/issues
   - Check existing issues for similar problems
   - Provide hardware details when asking for help

---

## Environment Variables Reference

The Python scripts now support environment variables for easier configuration:

```bash
# Serial port configuration
export LORA_SA_PORT=/dev/ttyUSB0      # LoRa device port
export DRONE_PORT=/dev/ttyACM0         # Drone/FC port (rpi-proxy-fc.py)
export SERIAL_BAUDRATE=115200          # Communication speed
export SERIAL_TIMEOUT=5                # Read timeout in seconds

# Run script with custom config
python3 ASCII_SA.py

# Or inline
LORA_SA_PORT=/dev/ttyUSB1 python3 SpectrumScan.py
```

---

## Advanced Debugging

### Memory Issues

If experiencing crashes or freezes:

1. **Check Memory Usage**:
   ```c
   Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
   ```

2. **Reduce Buffer Sizes**: Edit configuration to use less memory
3. **Disable Features**: Turn off waterfall, WiFi scanning to save memory

### Timing Issues

If scans appear delayed or unresponsive:

1. **Disable Watchdog**: May need to increase timeout
2. **Profile Code**: Add timing measurements
3. **Reduce Display Updates**: Update less frequently

### Radio Issues

If radio performance is poor:

1. **Check Antenna**: Measure VSWR if possible
2. **Verify Frequency**: Use SDR to confirm actual transmission frequency
3. **Test with Known Good Hardware**: Isolate hardware vs software issues
