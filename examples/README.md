# Examples

This directory contains example scripts demonstrating how to use the `micropython_msp` library.

## Available Examples

### `attitude_monitor.py`
Continuously requests and displays attitude data (roll, pitch, yaw) from a flight controller.

**Usage:**
1. Copy the `micropython_msp/` folder to your Pico filesystem
2. Copy `attitude_monitor.py` to your Pico
3. Run the script - it will continuously print attitude data to the REPL

**Configuration:**
- Default UART: UART(0) at 115200 baud
- Default pins: TX=GP0, RX=GP1
- Adjust these in the `main_loop()` function if needed

### `multi_data_monitor.py`
Requests and displays multiple types of data from a flight controller:
- **Attitude**: Roll, pitch, and yaw angles
- **Status**: Cycle time, I2C errors, active sensors, flight mode, profile
- **Analog**: Battery voltage, current draw, mAh consumed, RSSI

**Usage:**
1. Copy the `micropython_msp/` folder to your Pico filesystem
2. Copy `multi_data_monitor.py` to your Pico
3. Run the script - it will continuously print multiple data types to the REPL

**Configuration:**
- Default UART: UART(0) at 115200 baud
- Default pins: TX=GP0, RX=GP1
- Adjust these in the `main_loop()` function if needed

**Note:** This example uses the built-in parse methods from the library (`parse_status()`, `parse_analog()`, etc.). The `parse_analog()` method defaults to Betaflight format - pass `is_inav=True` if using iNAV firmware.

## Deploying to Pico

### Using mpremote:
```bash
# Copy the entire library folder
mpremote connect usb fs cp -r ../micropython_msp :/micropython_msp

# Copy example
mpremote connect usb fs put attitude_monitor.py :/attitude_monitor.py

# Run the example
mpremote connect usb run attitude_monitor.py
```

### Using Thonny:
1. Open Thonny and connect to your Pico
2. Copy the entire `micropython_msp/` folder from the project root to your Pico
3. Copy the example file you want to run
4. Run the script from Thonny

## Wiring

Connect your Pico to the flight controller:
- Pico TX (GP0) → FC RX
- Pico RX (GP1) → FC TX
- Common GND
- Use 3.3V logic levels
