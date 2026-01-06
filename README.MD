# MicroPython MSP Client Library

A lightweight MicroPython library for communicating with flight controllers (Betaflight, iNAV, etc.) using the Multiwii Serial Protocol (MSP) v1 over UART.

This is a general MicroPython library that should work on any MicroPython-compatible board with UART support. **Note:** This library has only been tested on Raspberry Pi Pico W and Betaglight FC.

## ⚠️ Warning

**This is a school project and may contain mistakes or bugs. Use at your own risk.** This library is provided as-is without any warranty. Always test thoroughly in a safe environment before using with actual flight controllers or drones. The authors are not responsible for any damage, injury, or issues resulting from the use of this software.

## Features

- Simple class-based API
- MSP v1 protocol support
- Built-in attitude parsing
- Easy to use and extend

## Project Structure

```
micropython-drone/
├── micropython_msp/      # Library package (copy this entire folder to your device)
│   ├── __init__.py       # Package initialization
│   ├── micropython_msp.py    # Main library file
│   └── MSP_commands.py   # MSP command constants
├── examples/             # Example scripts (configured for Pico W)
│   ├── attitude_monitor.py
│   ├── multi_data_monitor.py
│   └── README.md
└── README.MD            # This file
```

## Quick Start

### Installation

1. Copy the entire `micropython_msp/` folder to your MicroPython device filesystem
2. Import and use in your code:

```python
from machine import UART
from micropython_msp import MicroPythonMSP, MSP_ATTITUDE

uart = UART(0, baudrate=115200)
msp = MicroPythonMSP(uart)

msp.send_request(MSP_ATTITUDE)
cmd, payload = msp.read_response(timeout_ms=100)
angles = msp.parse_attitude(payload)
```

### Examples

See the [examples/](examples/) directory for complete working examples:
- `attitude_monitor.py` - Continuously monitors and displays attitude data (roll, pitch, yaw)
- `multi_data_monitor.py` - Monitors multiple data types: attitude, status, and analog (battery voltage, current, RSSI)

## Hardware Setup

### Wiring

Connect your MicroPython device to the flight controller:
- **Device TX** → **FC RX**
- **Device RX** → **FC TX**
- **Common GND**
- Use **3.3V logic levels** (or appropriate logic levels for your device)

**Note:** The examples use Raspberry Pi Pico W pin configuration (TX=GP0, RX=GP1). Adjust pins for your specific board.

### Configuration

- Default UART: `UART(0)` at 115200 baud (adjust for your board)
- Default pins (Pico W): TX=GP0, RX=GP1
- Adjust these in your code as needed for your specific board

## Deploying to Your Device

To use this library on your MicroPython-compatible device:

1. **Upload the entire `micropython_msp/` folder** to your device filesystem. This folder contains all the library files you need.

2. **Include the library in your code** by importing it:
   ```python
   from micropython_msp import MicroPythonMSP, MSP_ATTITUDE
   ```

3. **Use examples as a starting point**: Check out the example scripts in the `examples/` directory:
   - `examples/attitude_monitor.py` - Simple example showing how to monitor attitude data (configured for Pico W)
   - `examples/multi_data_monitor.py` - Advanced example showing multiple data types: attitude, status, and analog data
   
   You can upload and run these examples directly, or use them as a reference for your own projects. Note that the examples are configured for Raspberry Pi Pico W - adjust UART pins and configuration for your specific board.

You can upload the folder using any MicroPython tool like Thonny, mpremote, rshell, or ampy. Simply copy the entire `micropython_msp/` directory to your device.

## API Reference

### `MicroPythonMSP(uart)`

Main class for MSP communication.

**Parameters:**
- `uart`: UART object from `machine.UART`

**Methods:**
- `send_request(cmd, payload=b"")` - Send an MSP v1 request
- `read_response(timeout_ms=100)` - Read MSP response, returns `(cmd, payload)` or `(None, None)`

**Parse Methods:**
- `parse_attitude(payload_bytes)` - Parse MSP_ATTITUDE, returns `(roll, pitch, yaw)` in degrees
- `parse_status(payload_bytes)` - Parse MSP_STATUS, returns dict with cycle_time, i2c_errors, active_sensors, mode, profile
- `parse_analog(payload_bytes, is_inav=False)` - Parse MSP_ANALOG, returns dict with voltage, mah_drawn, rssi, amperage
- `parse_raw_imu(payload_bytes)` - Parse MSP_RAW_IMU, returns dict with accelerometer, gyroscope, magnetometer arrays
- `parse_altitude(payload_bytes)` - Parse MSP_ALTITUDE, returns altitude in meters (float)
- `parse_rc(payload_bytes)` - Parse MSP_RC, returns dict with active_channels and channels list
- `parse_motor(payload_bytes)` - Parse MSP_MOTOR, returns list of motor values (0-2000)
- `parse_servo(payload_bytes)` - Parse MSP_SERVO, returns list of servo values
- `parse_raw_gps(payload_bytes, is_inav=False)` - Parse MSP_RAW_GPS, returns dict with GPS data

## Troubleshooting

- **No data appears**: Confirm wiring and that the flight controller is sending MSP responses
- **Checksum errors**: Verify baud rate matches flight controller settings
- **Different scaling**: Some flight controllers use different scaling for attitude. Adjust `parse_attitude()` accordingly

## Acknowledgments

This library was inspired by [YAMSPy](https://github.com/thecognifly/YAMSPy), a Python implementation of the Multiwii Serial Protocol for standard Python environments.

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.
