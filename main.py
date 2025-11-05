import machine
import time
import struct
from MSP_commands import MSP_ATTITUDE

# UART used to talk to flight controller
# Default: UART0 TX=GP0, RX=GP1
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1), timeout=100)

def send_msp_request(cmd):
    # Build MSP request packet: "$M<" len cmd payload checksum
    packet = bytearray(b"$M<")
    packet.append(0)          # payload length = 0
    packet.append(cmd)        # command
    packet.append(cmd & 0xFF) # checksum for empty payload = cmd
    uart.write(packet)

def read_msp_response(timeout_ms=500):
    start = time.ticks_ms()
    buf = bytearray()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if uart.any():
            data = uart.read()
            if data:
                buf.extend(data)
            # look for header
            idx = buf.find(b"$M>")
            if idx != -1:
                # ensure header + len + cmd available
                if len(buf) >= idx + 5:
                    offset = idx + 3
                    length = buf[offset]
                    cmd = buf[offset + 1]
                    total_needed = offset + 2 + length + 1  # payload + checksum
                    # wait for full packet if not yet received
                    t0 = time.ticks_ms()
                    while len(buf) < total_needed and time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
                        if uart.any():
                            r = uart.read()
                            if r:
                                buf.extend(r)
                    if len(buf) >= total_needed:
                        payload = bytes(buf[offset + 2: offset + 2 + length])
                        checksum = buf[offset + 2 + length]
                        ch = length ^ cmd
                        for b in payload:
                            ch ^= b
                        if (ch & 0xFF) == checksum:
                            return cmd, payload
                        else:
                            return None, None
    return None, None

def parse_attitude(payload):
    # Typical MSP_ATTITUDE payload: 3 x int16 (roll, pitch, yaw)
    if len(payload) >= 6:
        roll, pitch, yaw = struct.unpack('<hhh', payload[:6])
        # many FCs send degrees * 10 -> convert to degrees
        return roll / 10.0, pitch / 10.0, yaw / 10.0
    return None

def main():
    print("Pico MSP Attitude client starting. Talking to FC via UART0 (GP0/GP1).")
    time.sleep(0.5)
    try:
        while True:
            send_msp_request(MSP_ATTITUDE)
            cmd, payload = read_msp_response(timeout_ms=300)
            if cmd == MSP_ATTITUDE and payload:
                vals = parse_attitude(payload)
                if vals:
                    roll, pitch, yaw = vals
                    # Print a simple CSV line for easy host parsing
                    print("ATT,{:.1f},{:.1f},{:.1f}".format(roll, pitch, yaw))
            # avoid flooding UART/FC
            time.sleep_ms(50)
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == "__main__":
    main()