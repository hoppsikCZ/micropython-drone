# MicroPython MSP Client Library

A lightweight MicroPython library for communicating with flight controllers (Betaflight, iNAV, etc.) using the Multiwii Serial Protocol (MSP) v1 over UART.

This is a general MicroPython library that should work on any MicroPython-compatible board with UART support. **Note:** This library has only been tested on Raspberry Pi Pico W.

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
- `attitude_monitor.py` - Continuously monitors and displays attitude data

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
   - `examples/attitude_monitor.py` - Complete example showing how to monitor attitude data (configured for Pico W)
   
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
- `parse_attitude(payload_bytes)` - Parse attitude data, returns `(roll, pitch, yaw)` in degrees

## Troubleshooting

- **No data appears**: Confirm wiring and that the flight controller is sending MSP responses
- **Checksum errors**: Verify baud rate matches flight controller settings
- **Different scaling**: Some flight controllers use different scaling for attitude. Adjust `parse_attitude()` accordingly

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.
