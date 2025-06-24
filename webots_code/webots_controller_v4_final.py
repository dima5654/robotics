from controller import Robot
import json
import socket
import time

# === TCP Socket Setup ===
ESP32_IP = '192.168.0.100'   # IP address of the ESP32
ESP32_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)

# Attempt connection to ESP32
try:
    sock.connect((ESP32_IP, ESP32_PORT))
    print("Connected to ESP32")
except Exception as e:
    print("Connection failed:", e)
    exit(1)

# Set read timeout for ESP32 responses
sock.settimeout(0.3)

# === Webots Initialization ===
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Robot speed constants
MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

# Counter for correcting line swings
counter = 0
COUNTER_MAX = 5

# Initial and goal node for ESP32 path planning
start_node = 'B4'
goal_node = 'S4'

# Possible movement states
states = ['forward', 'forward_bit', 'swing_right', 'swing_left', 
          'turn_right', 'turn_left', 'turn_back', 'stop', 'nothing']
current_state = 'forward'
message = ''

# Initialize ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# Initialize proximity sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# === Wait for ESP32 to be ready ===
esp32_ready = False
while robot.step(timestep) != -1 and not esp32_ready:
    try:
        msg = sock.recv(1024).decode().strip()
        if msg == 'esp32_ready':
            # Send start and goal to ESP32 as JSON
            sock.send((json.dumps({'start': start_node, 'goal': goal_node}) + '\n').encode())
            esp32_ready = True
    except:
        pass

# === Main Loop ===
while robot.step(timestep) != -1:
    # Read all sensors
    gsValues = [gs[i].getValue() for i in range(3)]
    psValues = [ps[i].getValue() for i in range(8)]

    # Encode ground sensor data
    message = ''.join(['0' if v > 600 else '1' for v in gsValues]) + '\n'

    # Detect obstacle using leftmost and rightmost proximity sensors
    obstacle_detected = psValues[0] > 300 or psValues[7] > 300

    if obstacle_detected:
        # Notify ESP32 and rotate to avoid obstacle
        sock.send(b'obstacle\n')
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(-0.5 * speed)
            rightMotor.setVelocity(0.5 * speed)
            if robot.getTime() - turn_start >= 3.52:
                break
        # Wait for ESP32 to confirm re-planning
        try:
            while True:
                response = sock.recv(1024).decode().strip()
                if response == 'replanned':
                    print("[INFO] ESP32 finished re-planning")
                    break
        except socket.timeout:
            print("Timeout waiting for re-planning confirmation")
        continue

    # Send current sensor reading to ESP32
    try:
        sock.send(message.encode())
    except:
        print("Send failed, ESP32 disconnected?")
        break

    # Receive new state from ESP32
    try:
        msg = sock.recv(1024).decode().strip()
        if msg in states:
            current_state = msg
        elif msg:
            print("ESP32 says:", msg)
    except:
        pass

    # Interpret line sensor values for swing correction
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600

    # Correct state based on line sensor logic
    if line_right and not line_left:
        current_state = 'swing_right'
    elif line_left and not line_right:
        current_state = 'swing_left'
    elif line_left and line_right and line_center:
        current_state = 'swing_left'

    # === Motor control logic ===
    if current_state == 'forward':
        counter = 0
        leftMotor.setVelocity(speed)
        rightMotor.setVelocity(speed)

    elif current_state == 'swing_right':
        leftMotor.setVelocity(0.5 * speed)
        rightMotor.setVelocity(0.0)
        if counter >= COUNTER_MAX:
            current_state = 'forward'

    elif current_state == 'swing_left':
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.5 * speed)
        if counter >= COUNTER_MAX:
            current_state = 'forward'

    elif current_state == 'stop':
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)

        # Wait for ESP32 to send final path
        try:
            msg = sock.recv(1024).decode().strip()
            if msg:
                try:
                    data = json.loads(msg)
                    if 'graph_path' in data:
                        graph_data = data['graph_path']
                        print("Final Path Taken:", ' -> '.join(graph_data))
                except Exception as e:
                    print("Failed to parse graph path:", e)
        except socket.timeout:
            print("Timeout while waiting for final path")

    counter += 1

    # === Special movement behaviors from ESP32 ===
    if current_state == 'forward_bit':
        # Short forward movement
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.8:
                break
        current_state = 'forward'
        sock.send(b'done\n')

    elif current_state == 'turn_right':
        # Right turn sequence
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:
                break
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(0.5 * speed)
            rightMotor.setVelocity(-0.5 * speed)
            if robot.getTime() - turn_start >= 1.76:
                break
        current_state = 'forward'
        sock.send(b'done\n')

    elif current_state == 'turn_left':
        # Left turn sequence
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:
                break
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(-0.5 * speed)
            rightMotor.setVelocity(0.5 * speed)
            if robot.getTime() - turn_start >= 1.76:
                break
        current_state = 'forward'
        sock.send(b'done\n')

    # Debug output
    print(f"Sensor: {message.strip()} - State: {current_state}")
