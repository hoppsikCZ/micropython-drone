"""
MicroPython MSP v1 client library.

Provides a simple class-based interface for sending MSP v1 requests
and reading responses via UART. Designed for RP2040 / Pico.

Copyright (C) 2024 MicroPython MSP Client Library

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Usage:
    from micropython_msp import MicroPythonMSP
    from machine import UART
    
    uart = UART(0, baudrate=115200)
    msp = MicroPythonMSP(uart)
    
    msp.send_request(MSP_ATTITUDE)
    cmd, payload = msp.read_response(timeout_ms=100)
    angles = msp.parse_attitude(payload)
"""
import time
from machine import UART

try:
    import ustruct as struct
except Exception:
    import struct

try:
    # Import from same package
    from .MSP_commands import MSP_ATTITUDE, MSP_STATUS
except Exception:
    # fallback numeric values if MSP_commands not present on device
    MSP_ATTITUDE = 108
    MSP_STATUS = 101


class MicroPythonMSP:
    """MicroPython MSP v1 client for communicating with flight controllers."""
    
    def __init__(self, uart):
        """
        Initialize MSP client with a UART object.
        
        Args:
            uart: UART object from machine.UART
        """
        self.uart = uart
    
    def _build_msp_v1(self, cmd, payload=b""):
        """
        Build MSP v1 request packet.
        
        Args:
            cmd: MSP command code
            payload: Optional payload bytes
            
        Returns:
            bytes: Complete MSP v1 packet
        """
        length = len(payload)
        buf = bytearray()
        buf.extend(b"$M<")
        buf.append(length & 0xFF)
        buf.append(cmd & 0xFF)
        if length:
            buf.extend(payload)
        # checksum is XOR of len, cmd and payload bytes
        ch = length ^ (cmd & 0xFF)
        for b in payload:
            ch ^= b
        buf.append(ch & 0xFF)
        return bytes(buf)
    
    def send_request(self, cmd, payload=b""):
        """
        Send an MSP v1 request.
        
        Args:
            cmd: MSP command code
            payload: Optional payload bytes
        """
        pkt = self._build_msp_v1(cmd, payload)
        self.uart.write(pkt)
    
    def read_response(self, timeout_ms=100):
        """
        Read an MSP v1 response from UART.
        
        Args:
            timeout_ms: Timeout in milliseconds
            
        Returns:
            tuple: (cmd, payload) or (None, None) on timeout/error
        """
        start = time.ticks_ms()
        buf = bytearray()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    buf.extend(data)

            # look for MSP v1 response header '$M>'
            idx = buf.find(b"$M>")
            if idx != -1:
                # need at least header + len + cmd + checksum
                if len(buf) < idx + 5:
                    # wait for more bytes
                    continue
                offset = idx + 3
                length = buf[offset]
                cmd = buf[offset + 1]
                total_needed = offset + 2 + length + 1
                # wait until full packet arrives or timeout
                if len(buf) < total_needed:
                    continue
                payload = bytes(buf[offset + 2: offset + 2 + length])
                checksum = buf[offset + 2 + length]
                # validate chksum
                ch = length ^ cmd
                for b in payload:
                    ch ^= b
                if (ch & 0xFF) == checksum:
                    return cmd, payload
                else:
                    # bad checksum, drop consumed bytes and continue
                    del buf[:total_needed]
                    return None, None

        return None, None
    
    def parse_attitude(self, payload_bytes):
        """
        Parse MSP_ATTITUDE payload and return (roll, pitch, yaw) in degrees.

        MSP v1 ATTITUDE payload = 3 x int16 (little-endian). Values are
        typically in 0.1 degrees (divide by 10.0).
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            tuple: (roll, pitch, yaw) in degrees, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 6:
            return None
        try:
            r, p, y = struct.unpack('<hhh', payload_bytes[:6])
            return (r / 10.0, p / 10.0, y)
        except Exception:
            return None
    
    def parse_status(self, payload_bytes):
        """
        Parse MSP_STATUS payload.
        
        Format: cycleTime (uint16), i2cError (uint16), activeSensors (uint16),
                mode (uint32), profile (uint8)
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            dict: Dictionary with status data, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 11:
            return None
        try:
            cycle_time, i2c_error, sensors, mode, profile = struct.unpack('<HHHIB', payload_bytes[:11])
            return {
                'cycle_time': cycle_time,
                'i2c_errors': i2c_error,
                'active_sensors': sensors,
                'mode': mode,
                'profile': profile
            }
        except Exception:
            return None
    
    def parse_analog(self, payload_bytes, is_inav=False):
        """
        Parse MSP_ANALOG payload.
        
        Betaflight format: voltage (uint8), mAhdrawn (uint16), rssi (uint16),
                           amperage (int16), voltage2 (uint16) = 9 bytes
        iNAV format: voltage (uint8), mAhdrawn (uint16), rssi (uint16),
                     amperage (int16) = 7 bytes
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            is_inav: True if using iNAV firmware (default: False for Betaflight)
            
        Returns:
            dict: Dictionary with analog data, or None on error
        """
        if is_inav:
            if not payload_bytes or len(payload_bytes) < 7:
                return None
            try:
                v1, mah, rssi, amp = struct.unpack('<BHHh', payload_bytes[:7])
                return {
                    'voltage': v1 / 10.0,
                    'mah_drawn': mah,
                    'rssi': rssi,
                    'amperage': amp / 100.0
                }
            except Exception:
                return None
        else:
            # Betaflight format
            if not payload_bytes or len(payload_bytes) < 9:
                return None
            try:
                v1, mah, rssi, amp, v2 = struct.unpack('<BHHhH', payload_bytes[:9])
                return {
                    'voltage': v2 / 100.0,  # Betaflight uses the 2-byte value
                    'mah_drawn': mah,
                    'rssi': rssi,
                    'amperage': amp / 100.0
                }
            except Exception:
                return None
    
    def parse_raw_imu(self, payload_bytes):
        """
        Parse MSP_RAW_IMU payload.
        
        Format: 9 x int16 (little-endian) = 18 bytes
        Order: accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            dict: Dictionary with IMU data, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 18:
            return None
        try:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = struct.unpack('<9h', payload_bytes[:18])
            return {
                'accelerometer': [acc_x, acc_y, acc_z],
                'gyroscope': [gyro_x, gyro_y, gyro_z],
                'magnetometer': [mag_x, mag_y, mag_z]
            }
        except Exception:
            return None
    
    def parse_altitude(self, payload_bytes):
        """
        Parse MSP_ALTITUDE payload.
        
        Format: altitude (int32, little-endian) in cm, divided by 100.0 for meters
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            float: Altitude in meters, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 4:
            return None
        try:
            alt_cm = struct.unpack('<i', payload_bytes[:4])[0]
            return round(alt_cm / 100.0, 2)
        except Exception:
            return None
    
    def parse_rc(self, payload_bytes):
        """
        Parse MSP_RC payload.
        
        Format: Variable number of uint16 values (little-endian), one per channel
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            dict: Dictionary with channel count and channel values, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 2:
            return None
        try:
            n_channels = len(payload_bytes) // 2
            channels = struct.unpack('<{}H'.format(n_channels), payload_bytes)
            return {
                'active_channels': n_channels,
                'channels': list(channels)
            }
        except Exception:
            return None
    
    def parse_motor(self, payload_bytes):
        """
        Parse MSP_MOTOR payload.
        
        Format: Variable number of uint16 values (little-endian), one per motor (0-2000)
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            list: List of motor values, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 2:
            return None
        try:
            n_motors = len(payload_bytes) // 2
            motors = struct.unpack('<{}H'.format(n_motors), payload_bytes)
            return list(motors)
        except Exception:
            return None
    
    def parse_servo(self, payload_bytes):
        """
        Parse MSP_SERVO payload.
        
        Format: Variable number of uint16 values (little-endian), one per servo
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            
        Returns:
            list: List of servo values, or None on error
        """
        if not payload_bytes or len(payload_bytes) < 2:
            return None
        try:
            n_servos = len(payload_bytes) // 2
            servos = struct.unpack('<{}H'.format(n_servos), payload_bytes)
            return list(servos)
        except Exception:
            return None
    
    def parse_raw_gps(self, payload_bytes, is_inav=False):
        """
        Parse MSP_RAW_GPS payload.
        
        Betaflight format: fix (uint8), numSat (uint8), lat (int32), lon (int32),
                          alt (uint16), speed (uint16), ground_course (uint16) = 15 bytes
        iNAV format: same as BF + hdop (uint16) = 17 bytes
        
        Args:
            payload_bytes: Raw payload bytes from MSP response
            is_inav: True if using iNAV firmware (default: False for Betaflight)
            
        Returns:
            dict: Dictionary with GPS data, or None on error
        """
        if is_inav:
            if not payload_bytes or len(payload_bytes) < 17:
                return None
            try:
                fix, num_sat, lat, lon, alt, speed, course, hdop = struct.unpack('<BBiiHHHH', payload_bytes[:17])
                return {
                    'fix': fix,
                    'num_sat': num_sat,
                    'lat': lat / 10000000.0,  # Convert from 1e7 format to degrees
                    'lon': lon / 10000000.0,
                    'alt': alt,
                    'speed': speed,
                    'ground_course': course,
                    'hdop': hdop
                }
            except Exception:
                return None
        else:
            if not payload_bytes or len(payload_bytes) < 15:
                return None
            try:
                fix, num_sat, lat, lon, alt, speed, course = struct.unpack('<BBiiHHH', payload_bytes[:15])
                return {
                    'fix': fix,
                    'num_sat': num_sat,
                    'lat': lat / 10000000.0,  # Convert from 1e7 format to degrees
                    'lon': lon / 10000000.0,
                    'alt': alt,
                    'speed': speed,
                    'ground_course': course
                }
            except Exception:
                return None
