import network
import socket
import time
import json
import select
from machine import UART, Pin
from micropython_msp import MicroPythonMSP

# --- Configuration ---
WIFI_SSID = "PicoDrone_Link"
WIFI_PASS = "flysafe123"
UART_ID = 0
UART_BAUD = 115200
# Default UART pins for Pico: TX=GP0, RX=GP1
UART_TX_PIN = 0 
UART_RX_PIN = 1

# --- MSP Command IDs ---
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110

# --- Setup UART & MSP ---
uart = UART(UART_ID, baudrate=UART_BAUD, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
msp = MicroPythonMSP(uart)

# --- Setup Wi-Fi Access Point ---
ap = network.WLAN(network.AP_IF)
ap.config(essid=WIFI_SSID, password=WIFI_PASS)
ap.active(True)

# Wait for AP to activate
while not ap.active() and ap.status() != 3:
    time.sleep(0.1)

print(f"AP Active. Connect to '{WIFI_SSID}'")
print(f"Web Server running at http://{ap.ifconfig()[0]}")

# --- Web Server Helper HTML ---
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pico Drone Monitor</title>
  <style>
    body { font-family: sans-serif; background: #222; color: #fff; text-align: center; }
    .card { background: #333; margin: 10px; padding: 15px; border-radius: 8px; }
    .val { font-size: 1.2em; font-weight: bold; color: #00d2ff; }

    /* Cube / Cuboid visualization (uses CSS variables for dimensions) */
    .scene { width: 260px; height: 260px; perspective: 800px; margin: 18px auto; }
    .cube-wrap { width: 200px; height: 200px; position: relative; margin: auto; transform-style: preserve-3d; }
    /* --w: width, --h: height, --d: depth (all full sizes) */
    .cube { --w: 100px; --h: 100px; --d: 100px; width: var(--w); height: var(--h); position: relative; margin: auto; transform-style: preserve-3d; transform: translateZ(0); transition: transform 0.12s ease-out; }
    /* cuboid preset: square base (w == d) and shorter height */
    .cube.cuboid { --w: 140px; --h: 80px; --d: 140px; }
    .face { position: absolute; box-sizing: border-box; border: 2px solid #00d2ff; background: rgba(0,210,255,0.06); }
    /* front/back use width x height */
    .face.front, .face.back { width: var(--w); height: var(--h); left: 0; top: 0; }
    .front  { transform: translateZ(calc(var(--d) / 2)); background: rgba(50,255,100,0.12); border-color: #3ae07a; }
    .back   { transform: rotateY(180deg) translateZ(calc(var(--d) / 2)); }
    /* right/left use depth x height (position centered horizontally) */
    .face.right, .face.left { width: var(--d); height: var(--h); top: 0; left: calc((var(--w) - var(--d)) / 2); }
    .right { transform: rotateY(90deg) translateZ(calc(var(--w) / 2)); }
    .left  { transform: rotateY(-90deg) translateZ(calc(var(--w) / 2)); }
    /* top/bottom use width x depth (position centered vertically) */
    .face.top, .face.bottom { width: var(--w); height: var(--d); left: 0; top: calc((var(--h) - var(--d)) / 2); }
    .top    { transform: rotateX(90deg) translateZ(calc(var(--h) / 2)); background: rgba(255,50,50,0.06); }
    .bottom { transform: rotateX(-90deg) translateZ(calc(var(--h) / 2)); }
    .info-grid { display:flex; gap:10px; justify-content:center; flex-wrap:wrap; }
    .mini { width:140px; }
  </style>
</head>
<body>
  <h2>Drone Status</h2>
  
  <div class="card">
    <div>Voltage: <span id="bat" class="val">--</span> V</div>
    <div>Amperage: <span id="amp" class="val">--</span> A</div>
  </div>

  <div class="card">
    <div>Roll: <span id="r" class="val">0</span>&deg;</div>
    <div>Pitch: <span id="p" class="val">0</span>&deg;</div>
    <div>Yaw: <span id="y" class="val">0</span>&deg;</div>
    
    <div class="scene">
      <div class="cube-wrap">
        <div class="cube cuboid" id="droneObj">
          <div class="face front"></div>
          <div class="face back"></div>
          <div class="face right"></div>
          <div class="face left"></div>
          <div class="face top"></div>
          <div class="face bottom"></div>
        </div>
      </div>
    </div>

    <div class="info-grid">
      <div class="card mini">Altitude: <span id="alt" class="val">--</span> m</div>
      <div class="card mini">Cycle: <span id="cycle" class="val">--</span> ms</div>
      <div class="card mini">I2C Err: <span id="i2c" class="val">--</span></div>
    </div>
  </div>

  <div class="card">
    <div>IMU Acc: <span id="acc" class="val">--, --, --</span></div>
    <div>IMU Gyro: <span id="gyr" class="val">--, --, --</span></div>
    <div>Motors: <span id="motvals" class="val">--</span></div>
  </div>

  <script>
    function update() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          // Update Text
          document.getElementById('r').innerText = (data.roll||0).toFixed(1);
          document.getElementById('p').innerText = (data.pitch||0).toFixed(1);
          document.getElementById('y').innerText = (data.yaw||0).toFixed(1);
          document.getElementById('bat').innerText = data.v || 0;
          document.getElementById('amp').innerText = data.a || 0;
          document.getElementById('alt').innerText = (data.alt!==undefined)?data.alt:"--";
          document.getElementById('cycle').innerText = data.cycle_time || "--";
          document.getElementById('i2c').innerText = data.i2c_errors || 0;

          // IMU
          if (data.imu) {
            let acc = data.imu.accelerometer || [0,0,0];
            let gyr = data.imu.gyroscope || [0,0,0];
            document.getElementById('acc').innerText = acc.join(', ');
            document.getElementById('gyr').innerText = gyr.join(', ');
          }

          // Motors
          if (data.motors && data.motors.length) {
            document.getElementById('motvals').innerText = data.motors.join(', ');
            for (let i=0;i<4;i++){
              const el = document.querySelector('.motor.m'+(i+1));
              if (!el) continue;
              const val = data.motors[i] || 0;
              // motor value typical 0-2000 -> normalize
              const scale = Math.min(1.6, 0.4 + (val/2000));
              el.style.transform = `translate(-50%,-50%) scale(${scale})`;
              el.title = val;
            }
          }

          // Update Drone transform (rotateX ~ pitch, rotateZ ~ roll)
          let drone = document.getElementById('droneObj');
          drone.style.transform = `rotateX(${-data.pitch||0}deg) rotateZ(${-data.roll||0}deg) rotateY(${data.yaw||0}deg)`;
        })
        .catch(err => console.log(err + " - Error fetching data"));
    }
    // Poll every 100ms
    setInterval(update, 100);
  </script>
</body>
</html>
"""

# --- Socket Setup ---
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(addr)
s.listen(1)
s.setblocking(False) # Non-blocking to allow MSP polling loop

# Shared Data State
drone_state = {
  "roll": 0, "pitch": 0, "yaw": 0,
  "v": 0, "a": 0,
  "motors": [0,0,0,0],
  "imu": {"accelerometer":[0,0,0], "gyroscope":[0,0,0]},
  "alt": 0,
  "cycle_time": 0,
  "i2c_errors": 0
}

print("System Ready. Loop starting.")

while True:
    # 1. Poll MSP Data (Attitude)
    msp.send_request(MSP_ATTITUDE)
    cmd, payload = msp.read_response(timeout_ms=20) # Short timeout to keep loop fast
    if cmd == MSP_ATTITUDE and payload:
        att = msp.parse_attitude(payload)
        if att:
            drone_state["roll"], drone_state["pitch"], drone_state["yaw"] = att

    # 2. Poll MSP Data (Analog/Battery) - Less frequent? 
    # For simplicity, we do it every loop, but you could add a counter to skip frames
    msp.send_request(MSP_ANALOG)
    cmd, payload = msp.read_response(timeout_ms=20)
    if cmd == MSP_ANALOG and payload:
        analog = msp.parse_analog(payload)
        if analog:
            drone_state["v"] = analog.get('voltage', 0)
            drone_state["a"] = analog.get('amperage', 0)

    # 2b. Poll MSP Data (Motors)
    msp.send_request(MSP_MOTOR)
    cmd, payload = msp.read_response(timeout_ms=20)
    if cmd == MSP_MOTOR and payload:
      motors = msp.parse_motor(payload)
      if motors:
        motors = list(motors)
        if len(motors) < 4:
          motors += [0] * (4 - len(motors))
        drone_state["motors"] = motors[:4]

    # 2c. Poll MSP Data (Raw IMU)
    msp.send_request(MSP_RAW_IMU)
    cmd, payload = msp.read_response(timeout_ms=20)
    if cmd == MSP_RAW_IMU and payload:
      rawimu = msp.parse_raw_imu(payload)
      if rawimu:
        drone_state["imu"] = rawimu

    # 2d. Poll MSP Data (Altitude)
    msp.send_request(MSP_ALTITUDE)
    cmd, payload = msp.read_response(timeout_ms=20)
    if cmd == MSP_ALTITUDE and payload:
      alt = msp.parse_altitude(payload)
      if alt is not None:
        drone_state["alt"] = alt

    # 2e. Poll MSP Data (Status)
    msp.send_request(MSP_STATUS)
    cmd, payload = msp.read_response(timeout_ms=20)
    if cmd == MSP_STATUS and payload:
      st = msp.parse_status(payload)
      if st:
        drone_state["cycle_time"] = st.get('cycle_time', drone_state.get('cycle_time', 0))
        drone_state["i2c_errors"] = st.get('i2c_errors', drone_state.get('i2c_errors', 0))

    # 3. Handle Web Server Requests
    try:
        # Check if a client is trying to connect
        res = select.select([s], [], [], 0)
        if res and len(res) > 0 and res[0]:  # Ensure res is not None and has elements
            cl, addr = s.accept()
            # Ensure the accepted client socket is blocking to avoid EAGAIN on send/recv
            try:
              cl.setblocking(True)
            except Exception:
              pass

            try:
              request = cl.recv(1024)
            except OSError as e_req:
              print("Client recv error:", e_req)
              cl.close()
              continue

            req_str = str(request)

            # Simple Routing
            if 'GET /data' in req_str:
              # Send JSON data
              print("Sending data:", drone_state)
              response = json.dumps(drone_state)
              header = 'HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n'
              try:
                cl.send(header + response)
              except OSError as e_send:
                print("Client send error:", e_send)
            else:
              # Send HTML Dashboard
              header = 'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n'
              try:
                cl.send(header + HTML_PAGE)
              except OSError as e_send:
                print("Client send error:", e_send)

            cl.close()
    except OSError as e:
        print("Socket error:", e)
    
    # Small sleep to prevent CPU hogging, though MSP timeout handles some delay
    time.sleep(0.01)