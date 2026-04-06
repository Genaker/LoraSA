#!/usr/bin/env python3
"""
Unit tests for serial_utils module.
Run with: python3 -m unittest test_serial_utils.py
"""

import unittest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from serial_utils import (
    crc16,
    parse_scan_result,
    parse_scan_result_regex,
    validate_serial_config
)


class TestCRC16(unittest.TestCase):
    """Tests for CRC16 calculation."""
    
    def test_crc16_basic(self):
        """Test basic CRC16 calculation."""
        result = crc16("test", 0)
        self.assertIsInstance(result, int)
        self.assertGreaterEqual(result, 0)
        self.assertLessEqual(result, 0xFFFF)
    
    def test_crc16_empty(self):
        """Test CRC16 with empty string."""
        result = crc16("", 0)
        # Empty string with initial value 0 produces 0
        self.assertEqual(result, 0)
    
    def test_crc16_consistency(self):
        """Test that same input gives same output."""
        input_str = "SCAN_RESULT 123"
        result1 = crc16(input_str, 0)
        result2 = crc16(input_str, 0)
        self.assertEqual(result1, result2)


class TestParseScanResult(unittest.TestCase):
    """Tests for parse_scan_result function."""
    
    def test_valid_input(self):
        """Test parsing valid SCAN_RESULT data."""
        line = "SCAN_RESULT 2 [(850000, -100), (860000, -90)]"
        count, data = parse_scan_result(line)
        
        self.assertEqual(count, 2)
        self.assertEqual(len(data), 2)
        self.assertEqual(data[0], [850000, -100])
        self.assertEqual(data[1], [860000, -90])
    
    def test_valid_with_garbage(self):
        """Test parsing with garbage before SCAN_RESULT."""
        line = "garbage data SCAN_RESULT 1 [(900000, -80)]"
        count, data = parse_scan_result(line)
        
        self.assertEqual(count, 1)
        self.assertEqual(len(data), 1)
    
    def test_missing_scan_result(self):
        """Test error handling for missing SCAN_RESULT."""
        with self.assertRaises(ValueError) as cm:
            parse_scan_result("no scan result here")
        self.assertIn("SCAN_RESULT", str(cm.exception))
    
    def test_invalid_format(self):
        """Test error handling for invalid format."""
        with self.assertRaises(ValueError):
            parse_scan_result("SCAN_RESULT")  # Missing count and data
    
    def test_count_mismatch(self):
        """Test error handling when count doesn't match data length."""
        line = "SCAN_RESULT 5 [(850000, -100)]"  # Says 5 but only 1 item
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        self.assertIn("does not match count", str(cm.exception))
    
    def test_invalid_frequency_low(self):
        """Test error handling for frequency too low."""
        line = "SCAN_RESULT 1 [(1000, -100)]"  # 1 kHz - too low
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        self.assertIn("Invalid frequency", str(cm.exception))
    
    def test_invalid_frequency_high(self):
        """Test error handling for frequency too high."""
        line = "SCAN_RESULT 1 [(9000000, -100)]"  # 9 GHz - too high
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        self.assertIn("Invalid frequency", str(cm.exception))
    
    def test_invalid_rssi_positive(self):
        """Test error handling for positive RSSI."""
        line = "SCAN_RESULT 1 [(850000, 50)]"  # Positive RSSI - invalid
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        self.assertIn("Invalid RSSI", str(cm.exception))
    
    def test_invalid_rssi_low(self):
        """Test error handling for RSSI too low."""
        line = "SCAN_RESULT 1 [(850000, -300)]"  # -300 dBm - too low
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        self.assertIn("Invalid RSSI", str(cm.exception))
    
    def test_count_too_large(self):
        """Test error handling for unreasonably large count."""
        # Create a line with a very large count value
        line = "SCAN_RESULT 99999 []"
        with self.assertRaises(ValueError) as cm:
            parse_scan_result(line)
        # Should fail either on count validation or count mismatch


class TestParseScanResultRegex(unittest.TestCase):
    """Tests for parse_scan_result_regex function."""
    
    def test_valid_input(self):
        """Test parsing valid data with regex method."""
        line = "SCAN_RESULT 2 [ (850000, -100), (860000, -90) ]"
        data = parse_scan_result_regex(line)
        
        self.assertEqual(len(data), 2)
        self.assertEqual(data[0], (850000, -100))
        self.assertEqual(data[1], (860000, -90))
    
    def test_empty_result(self):
        """Test error handling for empty result."""
        line = "SCAN_RESULT 0 []"
        with self.assertRaises(ValueError) as cm:
            parse_scan_result_regex(line)
        self.assertIn("No valid frequency/RSSI pairs", str(cm.exception))
    
    def test_missing_scan_result(self):
        """Test error handling when SCAN_RESULT is missing."""
        with self.assertRaises(ValueError):
            parse_scan_result_regex("just some data")


class TestValidateSerialConfig(unittest.TestCase):
    """Tests for validate_serial_config function."""
    
    def test_valid_config(self):
        """Test validation with valid configuration."""
        # Should not raise exception
        validate_serial_config("/dev/ttyUSB0", 115200, 5)
    
    def test_invalid_port_empty(self):
        """Test error handling for empty port."""
        with self.assertRaises(ValueError) as cm:
            validate_serial_config("", 115200, 5)
        self.assertIn("Port", str(cm.exception))
    
    def test_invalid_port_none(self):
        """Test error handling for None port."""
        with self.assertRaises(ValueError):
            validate_serial_config(None, 115200, 5)
    
    def test_invalid_baudrate(self):
        """Test error handling for invalid baudrate."""
        with self.assertRaises(ValueError) as cm:
            validate_serial_config("/dev/ttyUSB0", 12345, 5)
        self.assertIn("Baudrate", str(cm.exception))
    
    def test_invalid_timeout_negative(self):
        """Test error handling for negative timeout."""
        with self.assertRaises(ValueError) as cm:
            validate_serial_config("/dev/ttyUSB0", 115200, -1)
        self.assertIn("Timeout", str(cm.exception))
    
    def test_invalid_timeout_too_high(self):
        """Test error handling for timeout too high."""
        with self.assertRaises(ValueError) as cm:
            validate_serial_config("/dev/ttyUSB0", 115200, 500)
        self.assertIn("Timeout", str(cm.exception))
    
    def test_valid_baudrates(self):
        """Test that all common baudrates are accepted."""
        valid_baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
        for baudrate in valid_baudrates:
            validate_serial_config("/dev/ttyUSB0", baudrate, 5)


class TestIntegration(unittest.TestCase):
    """Integration tests combining multiple functions."""
    
    def test_full_parsing_pipeline(self):
        """Test complete parsing pipeline."""
        # Simulate real data from device
        raw_data = "other data SCAN_RESULT 3 [(850000, -110), (851000, -105), (852000, -100)]"
        
        # Parse the data
        count, data = parse_scan_result(raw_data)
        
        # Verify results
        self.assertEqual(count, 3)
        self.assertEqual(len(data), 3)
        
        # Check individual values
        for freq, rssi in data:
            self.assertGreaterEqual(freq, 100000)
            self.assertLessEqual(freq, 6000000)
            self.assertGreaterEqual(rssi, -200)
            self.assertLessEqual(rssi, 0)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
