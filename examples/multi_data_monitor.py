"""
MicroPython MSP v1 client example - Multiple data types monitor.

Requests and displays multiple types of data from a flight controller:
- Attitude (roll, pitch, yaw)
- Status (cycle time, mode, sensors)
- Analog (battery voltage, current, RSSI)

Designed for RP2040 / Pico W using UART(0) TX=GP0, RX=GP1 at 115200.

Usage:
  - Copy the micropython_msp folder to your Pico filesystem
  - Copy this file to your Pico and run it
  - Adjust UART pins/baud if needed.
"""
import time
from machine import UART
from micropython_msp import MicroPythonMSP, MSP_ATTITUDE, MSP_STATUS, MSP_ANALOG


def main_loop(uart_id=0, baudrate=115200):
    """Request multiple data types and print them."""
    uart = UART(uart_id, baudrate=baudrate, timeout=10)
    time.sleep_ms(50)
    
    # Initialize MSP client
    msp = MicroPythonMSP(uart)
    
    request_count = 0
    
    # Main loop: request different data types in sequence
    while True:
        request_count += 1
        print(f"\n--- Request #{request_count} ---")
        
        # Request and parse ATTITUDE
        msp.send_request(MSP_ATTITUDE)
        cmd, payload = msp.read_response(timeout_ms=100)
        if cmd == MSP_ATTITUDE:
            angles = msp.parse_attitude(payload)
            if angles:
                roll, pitch, yaw = angles
                print(f"Attitude: roll={roll:6.1f}°, pitch={pitch:6.1f}°, yaw={yaw:6.1f}°")
            else:
                print("Attitude: Parse error")
        else:
            print("Attitude: No response")
        
        time.sleep_ms(50)
        
        # Request and parse STATUS
        msp.send_request(MSP_STATUS)
        cmd, payload = msp.read_response(timeout_ms=100)
        if cmd == MSP_STATUS:
            status = msp.parse_status(payload)
            if status:
                print(f"Status: cycle={status['cycle_time']}us, "
                      f"i2c_err={status['i2c_errors']}, "
                      f"sensors=0x{status['active_sensors']:04X}, "
                      f"mode=0x{status['mode']:08X}, "
                      f"profile={status['profile']}")
            else:
                print("Status: Parse error")
        else:
            print("Status: No response")
        
        time.sleep_ms(50)
        
        # Request and parse ANALOG
        msp.send_request(MSP_ANALOG)
        cmd, payload = msp.read_response(timeout_ms=100)
        if cmd == MSP_ANALOG:
            analog = msp.parse_analog(payload)  # Default: Betaflight format
            if analog:
                print(f"Analog: voltage={analog['voltage']:.2f}V, "
                      f"current={analog['amperage']:.2f}A, "
                      f"mAh={analog['mah_drawn']}, "
                      f"RSSI={analog['rssi']}")
            else:
                print("Analog: Parse error")
        else:
            print("Analog: No response")
        
        # Wait before next cycle
        time.sleep_ms(500)


if __name__ == "__main__":
    try:
        main_loop()
    except Exception as e:
        print("Error:", e)
