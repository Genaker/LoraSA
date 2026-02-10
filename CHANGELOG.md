# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Python Utility Module**: Created `serial_utils.py` with shared functions for CRC16 calculation and data parsing
  - Eliminates code duplication across `ASCII_SA.py`, `SpectrumScan.py`, and `scripts/rpi-proxy-fc.py`
  - Provides centralized input validation and error handling
  - Includes backward compatibility fallbacks

- **Troubleshooting Guide**: Added comprehensive `TROUBLESHOOTING.md` with solutions for common issues
  - Build and compilation problems
  - Upload failures
  - Runtime errors
  - Configuration issues
  - Python script errors
  - Performance optimization tips
  - Environment variables reference

- **Environment Variable Support**: Python scripts now support configuration via environment variables
  - `LORA_SA_PORT`: Serial port for LoRa device
  - `DRONE_PORT`: Serial port for drone/flight controller
  - `SERIAL_BAUDRATE`: Communication baudrate
  - `SERIAL_TIMEOUT`: Read timeout in seconds

- **Signal Handlers**: Added graceful shutdown handling in `ASCII_SA.py`
  - Proper cleanup on Ctrl+C (SIGINT)
  - Prevents terminal corruption on exit

- **C++ Destructors**: Added proper destructors to prevent memory leaks
  - `StackedChart`: Cleans up dynamically allocated charts array
  - `BarChart`: Cleans up `ys` and `changed` arrays
  - Implements proper RAII pattern

### Changed

- **Exception Handling (Python)**:
  - Replaced bare `except:` clauses with specific exception types
  - Fixed incorrect `try-finally` logic in `scripts/rpi-proxy-fc.py`
  - Added proper error logging with descriptive messages
  - Improved error recovery and user feedback

- **Input Validation (Python)**:
  - Added comprehensive validation in `parse_scan_result()` functions
  - Frequency range validation (100 MHz to 6 GHz)
  - RSSI range validation (-200 to 0 dBm)
  - Scan count bounds checking (1 to 10,000)
  - Data integrity verification (count matches actual data length)

- **Memory Management (C++)**:
  - Fixed mixed allocation methods in `StackedChart.cpp`
  - Changed `free(charts)` to `delete[] charts` for consistency
  - Fixed BLE callback memory leak in `src/main.cpp`
  - Replaced heap allocation with static instance for `MyServerCallbacks`

- **Code Organization**:
  - Extracted hardcoded constants to configuration variables
  - Consolidated CRC16 implementations into single shared function
  - Improved code comments and documentation
  - Better separation of concerns in Python scripts

### Fixed

- **Python Scripts**:
  - Fixed infinite loops without proper exit mechanisms
  - Fixed silent exception swallowing with curses errors
  - Fixed Unicode decode errors in serial communication
  - Fixed malformed data handling in `ASCII_SA.py`

- **C++ Code**:
  - Fixed memory leaks in chart classes
  - Fixed undefined behavior from mixed allocation methods (new/free)
  - Fixed BLE server callback memory leak
  - Fixed potential buffer overflows from missing input validation

### Security

- **Input Validation**: Added bounds checking to prevent crashes from malformed serial data
- **Error Messages**: Improved error messages without exposing sensitive system information
- **Exception Handling**: Removed unsafe bare except clauses that could hide critical errors

### Documentation

- Added troubleshooting guide with common solutions
- Documented environment variable configuration
- Added examples for error handling
- Improved code comments in critical sections
- Added reference to troubleshooting guide in main README

### Performance

- Reduced memory allocations by using static instances where possible
- Improved error handling overhead by using specific exception types
- Better resource cleanup prevents memory leaks over time

---

## Notes for Developers

### Breaking Changes
None - all changes are backward compatible. Scripts will use local fallback implementations if `serial_utils.py` is not available.

### Migration Guide
No migration needed. Existing code continues to work. To use new features:

1. **Environment Variables**: Set before running scripts
   ```bash
   export LORA_SA_PORT=/dev/ttyUSB0
   python3 ASCII_SA.py
   ```

2. **Shared Utilities**: Import from `serial_utils` for new code
   ```python
   from serial_utils import crc16, parse_scan_result
   ```

### Testing Recommendations
- Test Python scripts with invalid/corrupted serial data
- Verify memory leak fixes with long-running tests
- Check backward compatibility with existing configurations
- Validate environment variable override functionality

---

## Future Improvements

Tracked in GitHub issues:
- [ ] Add unit tests for `serial_utils.py` validation functions
- [ ] Add integration tests for serial communication
- [ ] Create automated memory leak testing for C++ code
- [ ] Add configuration file support (YAML/JSON) for complex setups
- [ ] Implement logging framework for better debugging
- [ ] Add telemetry and metrics collection
