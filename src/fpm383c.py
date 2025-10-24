#!/usr/bin/env python3
"""
FPM383C Fingerprint Sensor Library for Raspberry Pi
Supports fingerprint image capture and template management
"""

import serial
import time
import struct
from typing import Optional, Tuple, List
import RPi.GPIO as GPIO

# Frame header
FP_FRAME_HEADER = bytes([0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A])

# Command categories
FP_CMD_FINGERPRINT_0 = 0x01
FP_CMD_SYSTEM_0 = 0x02
FP_CMD_MAINTENANCE_0 = 0x03

# Fingerprint commands
FP_CMD_ENROLL = 0x11
FP_CMD_QUERY_ENROLL = 0x12
FP_CMD_SAVE_TEMPLATE = 0x13
FP_CMD_QUERY_SAVE = 0x14
FP_CMD_CANCEL = 0x15
FP_CMD_UPDATE_FEATURE = 0x16
FP_CMD_QUERY_UPDATE = 0x17
FP_CMD_AUTO_ENROLL = 0x18
FP_CMD_MATCH = 0x21
FP_CMD_QUERY_MATCH = 0x22
FP_CMD_MATCH_SYNC = 0x23
FP_CMD_DELETE = 0x31
FP_CMD_QUERY_DELETE = 0x32
FP_CMD_CHECK_ID_EXIST = 0x33
FP_CMD_GET_STORAGE_INFO = 0x34
FP_CMD_CHECK_FINGER_STATUS = 0x35
FP_CMD_DELETE_SYNC = 0x36
FP_CMD_CONFIRM_ENROLL = 0x41
FP_CMD_QUERY_CONFIRM = 0x42
FP_CMD_UPLOAD_IMAGE = 0x02  # Fingerprint feature data upload
FP_CMD_FEATURE_INFORMATION = 0x01  # Fingerprint feature information upload

# System commands
FP_CMD_SET_PASSWORD = 0x01
FP_CMD_RESET_MODULE = 0x02
FP_CMD_GET_TEMPLATE_COUNT = 0x03
FP_CMD_GET_GAIN = 0x09
FP_CMD_GET_THRESHOLD = 0x0B
FP_CMD_SET_SLEEP_MODE = 0x0C
FP_CMD_SET_ENROLL_COUNT = 0x0D
FP_CMD_SET_LED = 0x0F
FP_CMD_GET_POLICY = 0xFB
FP_CMD_SET_POLICY = 0xFC

# Maintenance commands
FP_CMD_GET_MODULE_ID = 0x01
FP_CMD_HEARTBEAT = 0x03
FP_CMD_SET_BAUDRATE = 0x04
FP_CMD_SET_COMM_PASSWORD = 0x05

# Error codes
FP_ERROR_SUCCESS = 0x00000000
FP_ERROR_UNKNOWN_CMD = 0x00000001
FP_ERROR_INVALID_LENGTH = 0x00000002
FP_ERROR_INVALID_DATA = 0x00000003
FP_ERROR_SYSTEM_BUSY = 0x00000004
FP_ERROR_NO_REQUEST = 0x00000005
FP_ERROR_SOFTWARE_ERROR = 0x00000006
FP_ERROR_HARDWARE_ERROR = 0x00000007
FP_ERROR_TIMEOUT = 0x00000008
FP_ERROR_EXTRACTION_ERROR = 0x00000009
FP_ERROR_TEMPLATE_EMPTY = 0x0000000A
FP_ERROR_STORAGE_FULL = 0x0000000B
FP_ERROR_WRITE_FAILED = 0x0000000C
FP_ERROR_READ_FAILED = 0x0000000D
FP_ERROR_POOR_IMAGE = 0x0000000E
FP_ERROR_DUPLICATE = 0x0000000F
FP_ERROR_SMALL_AREA = 0x00000010
FP_ERROR_FINGERMOVEMENT_LARGE = 0x00000011 # Finger movement range is too large during image capture
FP_ERROR_FINGERMOVEMENT_SMALL = 0x00000012 # Finger movement range is too small during image capture
FP_ERROR_ID_OCCUPIED = 0x00000013 # Fingerprint ID is occupied
FP_ERROR_CAPTURE_FAILED = 0x00000014 # Fingerprint capture failed
FP_ERROR_COMMAND_FORCE_INTERRUPT = 0x00000015 # Command execution forced interruption
FP_ERROR_FEATURE_NOT_UPLOAD = 0x00000016 # Fingerprint feature data not uploaded
FP_ERROR_INVALID_ID = 0x00000017 # Invalid fingerprint ID
FP_ERROR_GAIN_ADJUSTMENT = 0x00000018 # Fingerprint gain adjustment error
FP_ERROR_BUFFER_OVERFLOW = 0x00000019 # Buffer overflow error
FP_ERROR_SLEEP_MODE = 0x0000001A # Fingerprint sleep mode error
FP_ERROR_CHECKSUM = 0x0000001C
FP_ERROR_WRITE_TO_FLASH = 0x00000022 # Write to flash memory failed
FP_ERROR_OTHER = 0x000000FF # Other errors
# LED colors
FP_LED_OFF = 0x00
FP_LED_GREEN = 0x01
FP_LED_RED = 0x02
FP_LED_RED_GREEN = 0x03
FP_LED_BLUE = 0x04
FP_LED_RED_BLUE = 0x05
FP_LED_GREEN_BLUE = 0x06
FP_LED_ALL_COLORS = 0x07

# LED control modes
FP_LED_MODE_OFF = 0x00
FP_LED_MODE_ON = 0x01
FP_LED_MODE_AUTO = 0x02
FP_LED_MODE_PWM = 0x03
FP_LED_MODE_BLINK = 0x04


class FingerprintMatchResult:
    def __init__(self, matched: bool = False, fingerprint_id: int = 0, match_score: int = 0):
        self.matched = matched
        self.fingerprint_id = fingerprint_id
        self.match_score = match_score


class FingerprintEnrollResult:
    def __init__(self, fingerprint_id: int = 0, progress: int = 0, completed: bool = False):
        self.fingerprint_id = fingerprint_id
        self.progress = progress
        self.completed = completed


class FingerprintStorageInfo:
    def __init__(self, total_count: int = 0, storage_map: bytes = b''):
        self.total_count = total_count
        self.storage_map = storage_map


class FPM383C:
    def __init__(self, port: str = '/dev/serial0', touch_pin: Optional[int] = None):
        """
        Initialize FPM383C fingerprint sensor
        
        Args:
            port: Serial port (default: /dev/serial0 for Raspberry Pi)
            touch_pin: GPIO pin for touch detection (optional)
        """
        self.serial = None
        self.port = port
        self.touch_pin = touch_pin
        self.password = 0x00000000
        self.last_error = FP_ERROR_SUCCESS
        self.debug_enabled = True
        
        if self.touch_pin is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def __del__(self):
        """Cleanup resources"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        if self.touch_pin is not None:
            GPIO.cleanup(self.touch_pin)
    
    def _debug_print(self, message: str):
        """Print debug message if debug is enabled"""
        if self.debug_enabled:
            print(f"[FPM383C] {message}")
    
    def _calculate_checksum(self, data: bytes) -> int:
        """Calculate checksum for data (two's complement)"""
        checksum = sum(data)
        return (~checksum + 1) & 0xFF
    
    def _calculate_frame_checksum(self, data_length: int) -> int:
        """Calculate frame checksum (two's complement)"""
        checksum = sum(FP_FRAME_HEADER)
        checksum += ((data_length >> 8) & 0xFF) + (data_length & 0xFF)
        return (~checksum + 1) & 0xFF
    
    def _send_frame(self, cmd1: int, cmd2: int, data: bytes = b''):
        """Send command frame to sensor"""
        data_length = len(data)
        password = [0x00, 0x00, 0x00, 0x00]  # Default password
        
        # Build application data (password + commands + data)
        appdata = bytearray()
        appdata.extend(password)
        appdata.append(cmd1)
        appdata.append(cmd2)
        appdata.extend(data)
        
        # Build frame: Header + Length + FrameChecksum + AppData + DataChecksum
        frame = bytearray()
        frame.extend(FP_FRAME_HEADER)  # 8 bytes header
        frame.append((data_length >> 8) & 0xFF)  # Length high byte
        frame.append(data_length & 0xFF)  # Length low byte
        frame.append(self._calculate_frame_checksum(data_length))  # Frame checksum
        frame.extend(appdata)  # Password + Commands + Data
        frame.append(self._calculate_checksum(appdata))  # Data checksum
        
        self._debug_print(f"Sending frame: {frame.hex()}")
        self.serial.write(bytes(frame))
        self.serial.flush()
    
    def _receive_frame(self, timeout: float = 5.0) -> Optional[Tuple[int, int, bytes, int]]:
        """
        Receive response frame from sensor
        Exact implementation matching C library
        
        Frame structure: 
        Header(8) + Length(2) + FrameChecksum(1) + AppData(dataLength)
        
        AppData structure:
        Password(4) + Cmd1(1) + Cmd2(1) + ErrorCode(4) + Data(n) + DataChecksum(1)
        
        Returns:
            Tuple of (cmd1, cmd2, data, error_code) or None if failed
        """
        start_time = time.time()
        timeout_seconds = timeout
        
        # Wait for frame header - exact C implementation logic
        header_idx = 0
        expected_header = list(FP_FRAME_HEADER)
        
        while time.time() - start_time < timeout_seconds and header_idx < 8:
            if self.serial.in_waiting > 0:
                byte = ord(self.serial.read(1))
                self._debug_print(f"Received byte: {hex(byte)}")
                if hex(byte) == expected_header[header_idx]:
                    header_idx += 1
                else:
                    header_idx = 0
                    if byte == expected_header[0]:
                        header_idx = 1
        
        if header_idx < 8:
            self.last_error = FP_ERROR_TIMEOUT
            self._debug_print("Timeout waiting for header")
            return None
        
        # Read data length (2 bytes)
        while time.time() - start_time < timeout_seconds and self.serial.in_waiting < 2:
            time.sleep(0.001)
        
        if self.serial.in_waiting < 2:
            self.last_error = FP_ERROR_TIMEOUT
            self._debug_print("Timeout reading length")
            return None
        
        length_bytes = self.serial.read(2)
        data_length = (length_bytes[0] << 8) | length_bytes[1]
        
        # Read frame checksum (1 byte)
        while time.time() - start_time < timeout_seconds and self.serial.in_waiting < 1:
            time.sleep(0.001)
        
        if self.serial.in_waiting < 1:
            self.last_error = FP_ERROR_TIMEOUT
            self._debug_print("Timeout reading frame checksum")
            return None
        
        frame_checksum = ord(self.serial.read(1))
        
        # Verify frame checksum
        expected_frame_checksum = self._calculate_frame_checksum(data_length)
        if frame_checksum != expected_frame_checksum:
            self.last_error = FP_ERROR_INVALID_DATA
            self._debug_print(f"Frame checksum mismatch: got {frame_checksum:02X}, expected {expected_frame_checksum:02X}")
            return None
        
        # Read application data (dataLength bytes)
        while time.time() - start_time < timeout_seconds and self.serial.in_waiting < data_length:
            time.sleep(0.001)
        
        if self.serial.in_waiting < data_length:
            self.last_error = FP_ERROR_TIMEOUT
            self._debug_print(f"Timeout reading app data (need {data_length}, have {self.serial.in_waiting})")
            return None
        
        app_data = bytearray(self.serial.read(data_length))
        
        # Verify application checksum (last byte of appData)
        received_checksum = app_data[data_length - 1]
        calculated_checksum = self._calculate_checksum(app_data[:data_length - 1])
        
        if received_checksum != calculated_checksum:
            self.last_error = FP_ERROR_INVALID_DATA
            self._debug_print(f"Data checksum mismatch: got {received_checksum:02X}, expected {calculated_checksum:02X}")
            return None
        
        # Parse response - exact C implementation
        # Bytes 0-3: Password (4 bytes)
        received_password = (app_data[0] << 24) | (app_data[1] << 16) | (app_data[2] << 8) | app_data[3]
        
        # Bytes 4-5: Commands (2 bytes)
        cmd1 = app_data[4]
        cmd2 = app_data[5]
        
        # Bytes 6-9: Error code (4 bytes)
        error_code = (app_data[6] << 24) | (app_data[7] << 16) | (app_data[8] << 8) | app_data[9]
        
        # Bytes 10 onwards: Actual response data (excluding last checksum byte)
        # responseDataLen = dataLength - 11 (4 password + 2 cmd + 4 error + 1 checksum)
        response_data_len = data_length - 11
        
        if response_data_len > 0:
            data = bytes(app_data[10:10 + response_data_len])
        else:
            data = b''
        
        self.last_error = error_code
        
        if self.debug_enabled:
            self._debug_print(f"Received response: cmd1={cmd1:02X}, cmd2={cmd2:02X}, error={error_code:08X}, data_len={len(data)}")
        
        return (cmd1, cmd2, data, error_code)
    
    def _send_command(self, cmd1: int, cmd2: int, data: bytes = b'') -> bool:
        """Send command and return success"""
        self._send_frame(cmd1, cmd2, data)
        return True
    
    def _receive_response(self, cmd1: int, cmd2: int, timeout: float = 5.0) -> Optional[Tuple[bytes, int]]:
        """
        Receive response for command
        
        Returns:
            Tuple of (data, error_code) or None if failed
        """
        response = self._receive_frame(timeout)
        if response is None:
            return None
        
        resp_cmd1, resp_cmd2, data, error_code = response
        
        if resp_cmd1 != cmd1 or resp_cmd2 != cmd2:
            self._debug_print(f"Command mismatch: expected {cmd1:02X}/{cmd2:02X}, got {resp_cmd1:02X}/{resp_cmd2:02X}")
            return None
        
        self.last_error = error_code
        return (data, error_code)
    
    def begin(self, baudrate: int = 57600) -> bool:
        """
        Initialize sensor communication
        
        Args:
            baudrate: Serial communication speed (default: 57600)
        
        Returns:
            True if successful
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            time.sleep(0.1)
            
            # Test connection with heartbeat
            return self.heartbeat()
        except Exception as e:
            self._debug_print(f"Failed to open serial port: {e}")
            return False
    
    def heartbeat(self) -> bool:
        """Send heartbeat to check sensor connection"""
        self._send_command(FP_CMD_MAINTENANCE_0, FP_CMD_HEARTBEAT)
        response = self._receive_response(FP_CMD_MAINTENANCE_0, FP_CMD_HEARTBEAT)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def reset(self) -> bool:
        """Reset the sensor module"""
        self._send_command(FP_CMD_SYSTEM_0, FP_CMD_RESET_MODULE)
        time.sleep(1.0)
        return True
    
    def set_led(self, mode: int, color: int, param1: int = 0, param2: int = 0, param3: int = 0) -> bool:
        """
        Control sensor LED
        
        Args:
            mode: LED mode (FP_LED_MODE_*)
            color: LED color (FP_LED_*)
            param1-3: Additional parameters for specific modes
        """
        data = bytes([mode, color, param1, param2, param3])
        self._send_command(FP_CMD_SYSTEM_0, FP_CMD_SET_LED, data)
        response = self._receive_response(FP_CMD_SYSTEM_0, FP_CMD_SET_LED)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def get_feature_information(self) -> Optional[bytes]:
        """
        Get fingerprint feature information
        
        Returns:
            Feature data or None if failed
        """
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_FEATURE_INFORMATION)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_FEATURE_INFORMATION, timeout=5.0)
        
        if response is None or response[1] != FP_ERROR_SUCCESS:
            return None
        
        return response[0][4:]  # Skip error code bytes
    
    def upload_fingerprint_image(self) -> Optional[bytes]:
        """
        Upload fingerprint image from sensor
        
        Returns:
            Image data or None if failed
        """
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_UPLOAD_IMAGE)
        
        # Image data comes in multiple packets
        image_data = bytearray()
        
        while True:
            response = self._receive_frame(timeout=5.0)
            if response is None:
                break
            
            cmd1, cmd2, data, error_code = response
            
            if cmd1 != FP_CMD_FINGERPRINT_0 or cmd2 != FP_CMD_UPLOAD_IMAGE:
                break
            
            if error_code != FP_ERROR_SUCCESS:
                self._debug_print(f"Image upload error: {error_code:08X}")
                return None
            
            # Skip the error code bytes and append image data
            image_data.extend(data[4:])
            
            # Check if this is the last packet (you may need to adjust this logic)
            if len(data) < 512:  # Assuming packets are typically 512 bytes
                break
        
        return bytes(image_data) if len(image_data) > 0 else None
    
    def start_match(self) -> bool:
        """Start fingerprint matching operation"""
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_MATCH)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_MATCH)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def query_match_result(self) -> FingerprintMatchResult:
        """Query the result of fingerprint matching"""
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_QUERY_MATCH)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_QUERY_MATCH)
        
        result = FingerprintMatchResult()
        
        if response is None or response[1] != FP_ERROR_SUCCESS:
            return result
        
        data = response[0]
        if len(data) >= 8:
            # Skip error code (4 bytes)
            result.fingerprint_id = struct.unpack('>H', data[4:6])[0]
            result.match_score = struct.unpack('>H', data[6:8])[0]
            result.matched = True
        
        return result
    
    def match_sync(self) -> FingerprintMatchResult:
        """Synchronous fingerprint matching"""
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_MATCH_SYNC)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_MATCH_SYNC, timeout=10.0)
        
        result = FingerprintMatchResult()
        
        if response is None or response[1] != FP_ERROR_SUCCESS:
            return result
        
        data = response[0]
        if len(data) >= 8:
            result.fingerprint_id = struct.unpack('>H', data[4:6])[0]
            result.match_score = struct.unpack('>H', data[6:8])[0]
            result.matched = True
        
        return result
    
    def auto_enroll(self, fingerprint_id: int, enroll_count: int = 6) -> bool:
        """
        Automatic fingerprint enrollment
        
        Args:
            fingerprint_id: ID to assign to fingerprint
            enroll_count: Number of scans required
        """
        data = struct.pack('>HB', fingerprint_id, enroll_count)
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_AUTO_ENROLL, data)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_AUTO_ENROLL, timeout=30.0)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def delete_fingerprint(self, fingerprint_id: int) -> bool:
        """Delete a fingerprint by ID"""
        data = struct.pack('>H', fingerprint_id)
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_DELETE, data)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_DELETE)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def delete_all_fingerprints(self) -> bool:
        """Delete all stored fingerprints"""
        data = struct.pack('>HH', 0, 0xFFFF)
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_DELETE, data)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_DELETE)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def get_template_count(self) -> int:
        """Get number of stored fingerprints"""
        self._send_command(FP_CMD_SYSTEM_0, FP_CMD_GET_TEMPLATE_COUNT)
        response = self._receive_response(FP_CMD_SYSTEM_0, FP_CMD_GET_TEMPLATE_COUNT)
        
        if response is None or response[1] != FP_ERROR_SUCCESS:
            return 0
        
        data = response[0]
        if len(data) >= 6:
            return struct.unpack('>H', data[4:6])[0]
        return 0
    
    def check_fingerprint_exists(self, fingerprint_id: int) -> bool:
        """Check if fingerprint ID exists"""
        data = struct.pack('>H', fingerprint_id)
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_CHECK_ID_EXIST, data)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_CHECK_ID_EXIST)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def is_finger_present(self) -> bool:
        """Check if finger is present on sensor"""
        if self.touch_pin is not None:
            return GPIO.input(self.touch_pin) == GPIO.LOW
        
        # Alternative: use sensor command
        self._send_command(FP_CMD_FINGERPRINT_0, FP_CMD_CHECK_FINGER_STATUS)
        response = self._receive_response(FP_CMD_FINGERPRINT_0, FP_CMD_CHECK_FINGER_STATUS, timeout=0.5)
        return response is not None and response[1] == FP_ERROR_SUCCESS
    
    def wait_for_finger(self, timeout: float = 10.0) -> bool:
        """Wait for finger to be placed on sensor"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_finger_present():
                return True
            time.sleep(0.1)
        return False
    
    def wait_for_finger_removal(self, timeout: float = 5.0) -> bool:
        """Wait for finger to be removed from sensor"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_finger_present():
                return True
            time.sleep(0.1)
        return False
    
    def get_last_error(self) -> int:
        """Get last error code"""
        return self.last_error
    
    def get_error_string(self, error_code: int) -> str:
        """Get human-readable error message"""
        error_messages = {
            FP_ERROR_SUCCESS: "Success",
            FP_ERROR_UNKNOWN_CMD: "Unknown command",
            FP_ERROR_INVALID_LENGTH: "Invalid length",
            FP_ERROR_INVALID_DATA: "Invalid data",
            FP_ERROR_SYSTEM_BUSY: "System busy",
            FP_ERROR_NO_REQUEST: "No request",
            FP_ERROR_SOFTWARE_ERROR: "Software error",
            FP_ERROR_HARDWARE_ERROR: "Hardware error",
            FP_ERROR_TIMEOUT: "Timeout",
            FP_ERROR_EXTRACTION_ERROR: "Feature extraction error",
            FP_ERROR_TEMPLATE_EMPTY: "Template empty",
            FP_ERROR_STORAGE_FULL: "Storage full",
            FP_ERROR_WRITE_FAILED: "Write failed",
            FP_ERROR_READ_FAILED: "Read failed",
            FP_ERROR_POOR_IMAGE: "Poor image quality",
            FP_ERROR_DUPLICATE: "Duplicate fingerprint",
            FP_ERROR_SMALL_AREA: "Fingerprint area too small",
        }
        return error_messages.get(error_code, f"Unknown error: {error_code:08X}")
    
    def enable_debug(self, enable: bool):
        """Enable or disable debug output"""
        self.debug_enabled = enable