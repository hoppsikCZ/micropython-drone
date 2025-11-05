# Raspberry Pi Pico — MSP Attitude client

This repo contains a MicroPython MSP client for Raspberry Pi Pico that requests MSP_ATTITUDE from a flight controller over a UART and prints roll/pitch/yaw to the Pico USB REPL.

Important notes
- This is a small MSP client implemented in MicroPython.
- Wiring: connect Pico TX -> FC RX, Pico RX -> FC TX, and common GND. Use 3.3V logic levels.
- Default pins: UART0 TX=GP0, RX=GP1. Change pins in `main.py` if needed.
- Baudrate: 115200 (adjust if your FC uses a different baud).

Files
- main.py — MicroPython program for the Pico. Prints lines like `ATT,roll,pitch,yaw` to the USB REPL.
- MSP_commands - constats of all possible MSP commands.

Deploy to Pico
1. Use Thonny, rshell, mpremote, or ampy to copy `main.py` to the Pico filesystem (as `/main.py` to auto-run).
   Example with mpremote:
   - mpremote connect usb fs put main.py :/main.py
2. Reboot Pico. The script will run and print attitude lines to the REPL.

Troubleshooting
- If no data appears, confirm wiring and that the flight controller is sending MSP responses to requests.
- Some flight controllers use different scaling for attitude (e.g. degrees×100). Adjust `parse_attitude()` in `main.py` accordingly.