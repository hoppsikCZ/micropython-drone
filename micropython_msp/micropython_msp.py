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
