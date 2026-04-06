# Contributing to LoraSA

Thank you for your interest in contributing to LoraSA! This document provides guidelines and best practices for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Focus on what is best for the community
- Show empathy towards other community members

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/LoraSA.git
   cd LoraSA
   ```
3. **Create a branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```

## Development Setup

### For C++ Development (ESP32)
1. Install VSCode and PlatformIO extension
2. Open project in VSCode
3. Select appropriate environment from `platformio.ini`
4. Build and upload to your board

### For Python Development
1. Install Python 3.7 or higher
2. Install dependencies:
   ```bash
   pip install pyserial matplotlib numpy
   ```
3. Test scripts from repository root:
   ```bash
   python3 ASCII_SA.py
   python3 SpectrumScan.py
   ```

## Code Quality Standards

### Python Code

#### Style Guidelines
- Follow PEP 8 style guide
- Use meaningful variable names
- Maximum line length: 100 characters
- Use type hints for function parameters and returns (Python 3.7+)

#### Best Practices
1. **Error Handling**:
   ```python
   # ❌ Bad - bare except
   try:
       risky_operation()
   except:
       pass
   
   # ✅ Good - specific exceptions
   try:
       risky_operation()
   except (ValueError, IOError) as e:
       logger.error(f"Operation failed: {e}")
       # Handle or re-raise
   ```

2. **Input Validation**:
   ```python
   # ❌ Bad - no validation
   def process_data(freq, rssi):
       return freq * rssi
   
   # ✅ Good - validate inputs
   def process_data(freq, rssi):
       if not (100000 <= freq <= 6000000):
           raise ValueError(f"Invalid frequency: {freq}")
       if not (-200 <= rssi <= 0):
           raise ValueError(f"Invalid RSSI: {rssi}")
       return freq * rssi
   ```

3. **Resource Management**:
   ```python
   # ❌ Bad - manual cleanup
   ser = serial.Serial(port, baudrate)
   data = ser.read()
   ser.close()
   
   # ✅ Good - context manager
   with serial.Serial(port, baudrate) as ser:
       data = ser.read()
   # Automatically closed
   ```

4. **Constants**:
   ```python
   # ❌ Bad - magic numbers
   if timeout > 5:
       ...
   
   # ✅ Good - named constants
   DEFAULT_TIMEOUT = 5
   if timeout > DEFAULT_TIMEOUT:
       ...
   ```

### C++ Code

#### Style Guidelines
- Follow existing code style in the project
- Use 4 spaces for indentation (no tabs)
- Maximum line length: 100 characters
- Use `clang-format` with provided `.clang-format` configuration

#### Best Practices

1. **Memory Management**:
   ```cpp
   // ❌ Bad - memory leak
   MyClass* obj = new MyClass();
   // Never deleted
   
   // ✅ Good - RAII or manual cleanup
   std::unique_ptr<MyClass> obj(new MyClass());
   // Or with destructor:
   ~MyClass() {
       delete[] dynamicArray;
   }
   ```

2. **Array Allocation**:
   ```cpp
   // ❌ Bad - mixed allocation
   char* arr = new char[100];
   free(arr);  // Wrong!
   
   // ✅ Good - consistent allocation
   char* arr = new char[100];
   delete[] arr;
   ```

3. **Null Checks**:
   ```cpp
   // ❌ Bad - no null check
   void process(Data* data) {
       data->value = 10;  // Crash if data is null
   }
   
   // ✅ Good - validate pointer
   void process(Data* data) {
       if (data == nullptr) {
           LOG("Error: null pointer");
           return;
       }
       data->value = 10;
   }
   ```

4. **Destructors**:
   ```cpp
   // ❌ Bad - missing destructor
   class MyChart {
       float* data;
   public:
       MyChart() { data = new float[100]; }
       // No destructor - memory leak!
   };
   
   // ✅ Good - proper cleanup
   class MyChart {
       float* data;
   public:
       MyChart() { data = new float[100]; }
       ~MyChart() { delete[] data; }
   };
   ```

## Testing

### Python Tests
Run existing tests:
```bash
cd test
python -m unittest test_rssi.py
```

Add tests for new features:
```python
import unittest
from serial_utils import parse_scan_result

class TestParseFunction(unittest.TestCase):
    def test_valid_input(self):
        line = "SCAN_RESULT 2 [(850000, -100), (860000, -90)]"
        count, data = parse_scan_result(line)
        self.assertEqual(count, 2)
        self.assertEqual(len(data), 2)
    
    def test_invalid_frequency(self):
        line = "SCAN_RESULT 1 [(1, -100)]"  # Freq too low
        with self.assertRaises(ValueError):
            parse_scan_result(line)
```

### C++ Tests
Tests are in `test/` directory. Run with PlatformIO:
```bash
pio test
```

## Pull Request Process

1. **Update Documentation**:
   - Update README.md if adding features
   - Add entry to CHANGELOG.md
   - Update TROUBLESHOOTING.md if relevant

2. **Test Your Changes**:
   - Build and test on actual hardware if possible
   - Run existing test suite
   - Test edge cases and error conditions

3. **Commit Messages**:
   ```
   Short summary (50 chars or less)
   
   More detailed explanation if needed. Wrap at 72 characters.
   
   - Bullet points are okay
   - Explain what and why, not how
   
   Fixes #123
   ```

4. **Create Pull Request**:
   - Describe changes clearly
   - Reference related issues
   - Include testing steps
   - Add screenshots for UI changes

5. **Code Review**:
   - Address reviewer feedback
   - Keep discussions focused and professional
   - Be open to suggestions

## Common Pitfalls to Avoid

### Python
- ❌ Bare `except:` clauses
- ❌ Magic numbers without constants
- ❌ Missing input validation
- ❌ Ignoring exceptions with `pass`
- ❌ Hardcoded paths/ports

### C++
- ❌ Memory leaks (new without delete)
- ❌ Mixed allocation (new/free, malloc/delete)
- ❌ Missing destructors for classes with dynamic allocation
- ❌ Null pointer dereferences
- ❌ Buffer overflows from unchecked indices

## Security Guidelines

1. **Never commit**:
   - Credentials or API keys
   - Personal information
   - Binary files (unless necessary)

2. **Input Validation**:
   - Always validate external inputs
   - Check array bounds
   - Validate ranges for frequencies, RSSI, etc.

3. **Error Messages**:
   - Don't expose internal paths
   - Don't reveal system information
   - Provide helpful but safe messages

## Documentation

- Comment complex algorithms
- Use docstrings for Python functions
- Update README for new features
- Add examples for new functionality

## Performance Considerations

- Avoid allocations in tight loops
- Use appropriate data structures
- Profile before optimizing
- Consider memory constraints of ESP32

## Hardware Testing

If possible, test on:
- Heltec WiFi LoRa 32 V3
- LilyGo T3S3 boards
- Different frequency ranges
- Various operating conditions

## Getting Help

- Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- Search existing issues
- Ask questions in issue comments
- Be specific about your setup

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE.md).

---

Thank you for contributing to LoraSA! 🎉
