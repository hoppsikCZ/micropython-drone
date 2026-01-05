"""
Minimal MicroPython MSP v1 client example.

Sends MSP requests (v1) and prints the response.
Designed for RP2040 / Pico using UART(0) TX=GP0, RX=GP1 at 115200.

Usage:
  - Copy the micropython_msp folder to your Pico filesystem
  - Copy this file to your Pico and run it
  - Adjust UART pins/baud if needed.
"""
import time
from machine import UART
from micropython_msp import MicroPythonMSP, MSP_ATTITUDE, MSP_STATUS


def main_loop(uart_id=0, baudrate=115200):
    """Open UART, send MSP_ATTITUDE request, print response."""
    uart = UART(uart_id, baudrate=baudrate, timeout=10)
    time.sleep_ms(50)
    
    # Initialize MSP client
    msp = MicroPythonMSP(uart)
    
    # Main loop: send requests and print responses
    while True:
        msp.send_request(MSP_ATTITUDE)
        cmd, payload = msp.read_response(timeout_ms=10)
        
        if cmd is None:
            print("No valid response or checksum error")
            continue
        
        print("Received cmd:", cmd)

        angles = msp.parse_attitude(payload)
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
