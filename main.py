"""
Minimal MicroPython MSP v1 client example.

Sends a single MSP request (v1) and prints the response.
Designed for RP2040 / Pico using UART(0) TX=GP0, RX=GP1 at 115200.

Usage:
  - Put this file on your Pico and run it (or import the functions from REPL)
  - Adjust UART pins/baud if needed.
"""
import time
from machine import UART

try:
    import ustruct as struct
except Exception:
    import struct

try:
    # prefer project MSP constants if available
    from MSP_commands import MSP_ATTITUDE, MSP_STATUS
except Exception:
    # fallback numeric values if MSP_commands not present on device
    MSP_ATTITUDE = 108
    MSP_STATUS = 101


def _build_msp_v1(cmd, payload=b""):
    """Return a bytes object for MSP v1 request (no payload typical for requests)."""
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


def send_msp_request(uart, cmd, payload=b""):
    pkt = _build_msp_v1(cmd, payload)
    uart.write(pkt)


def read_msp_response(uart, timeout_ms=100):
    """Read an MSP v1 response and return (cmd, payload) or (None, None) on timeout/error."""
    start = time.ticks_ms()
    buf = bytearray()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if uart.any():
            data = uart.read()
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

def parse_attitude(payload_bytes):
        """Parse MSP_ATTITUDE payload and return (roll, pitch, yaw) in degrees.

        MSP v1 ATTITUDE payload = 3 x int16 (little-endian). Values are
        typically in 0.1 degrees (divide by 10.0).
        """
        if not payload_bytes or len(payload_bytes) < 6:
            return None
        try:
            r, p, y = struct.unpack('<hhh', payload_bytes[:6])
            return (r / 10.0, p / 10.0, y)
        except Exception:
            return None

def main_loop(uart_id=0, baudrate=115200):
    """Open UART, send MSP_ATTITUDE request, print response."""
    uart = UART(uart_id, baudrate=baudrate, timeout=10)
    time.sleep_ms(50)
    # print("Sending MSP request...")
    while True:
        send_msp_request(uart, MSP_ATTITUDE)
        cmd, payload = read_msp_response(uart, timeout_ms=10)
        if cmd is None:
            print("No valid response or checksum error")
            continue
        print("Received cmd:", cmd)

        angles = parse_attitude(payload)
        if angles:
            roll, pitch, yaw = angles
            print(f"ATT: roll={roll}°, pitch={pitch}°, yaw={yaw}°")
        else:
            payload_len = len(payload) if payload is not None else 0
            print("Payload (len={}):".format(payload_len), payload)


if __name__ == "__main__":
    # run demo on import/run
    try:
        main_loop()
    except Exception as e:
        print("Error in demo:", e)
