from controller import Robot
import json
import socket
import time

# --- Socket TCP client to ESP32 ---
ESP32_IP = '192.168.0.101'  # IP for HARd router
#ESP32_IP = '10.149.34.96' # IP for ZP11
ESP32_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)
try:
    sock.connect((ESP32_IP, ESP32_PORT))
    print("Connected to ESP32")
except Exception as e:
    print("Connection failed:", e)
    exit(1)
#sock.settimeout(0.05)
sock.settimeout(0.2)

# --- Webots Initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

start_node = 'B4'
goal_node = 'L'

states = ['forward', 'forward_bit', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'
message =''

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# --- Wait for ESP32 ready ---
ready = False
while robot.step(timestep) != -1 and not ready:
    try:
        msg = sock.recv(1024).decode().strip()
        if msg == 'ready':
            sock.send((json.dumps({'start': start_node, 'goal': goal_node}) + '\n').encode())
            ready = True
    except:
        pass

path = []
path_index = 0

while robot.step(timestep) != -1:
    if current_state == 'forward':
        gsValues = [gs[i].getValue() for i in range(3)]
        message = ''.join(['0' if v > 600 else '1' for v in gsValues]) + '\n'
        #Check if sensor data was sent
        try:
            sock.send(message.encode())
        except:
            print("Send failed, ESP32 disconnected?")
            break
    #Check if esp32 message gives state to switch
    try:
        msg = sock.recv(1024).decode().strip()
        if msg.startswith('{') and 'path' in msg:
            data = json.loads(msg)
        elif msg in states:
            current_state = msg
        elif msg:
            print("ESP32 says:", msg)
    except Exception as e:
        print("Receive error:", e)
        
    #FORWARD
    if current_state == 'forward':
        leftMotor.setVelocity(speed)
        rightMotor.setVelocity(speed)
        
    elif current_state == 'forward_bit':
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.8:  # Adjust this value!
                break
        current_state = 'forward'
        sock.send(b'done\n')  # <- Notify ESP32
    
    #LEFT TURN    
    elif current_state == 'turn_right':
        #For moving bit forward
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:  # Adjust this value!
                break
        #For moving to the left
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(0.5 * speed)
            rightMotor.setVelocity(-0.5 * speed)
            if robot.getTime() - turn_start >= 2.2:  # Adjust this value!
                break
        current_state = 'forward'
        sock.send(b'done\n')  # <- Notify ESP32
    #RIGHT TURN
    elif current_state == 'turn_left':
        #For moving bit forward
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:  # Adjust this value!
                break
        #For moving to the right
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(-0.5 * speed)
            rightMotor.setVelocity(0.5 * speed)
            if robot.getTime() - turn_start >= 1.76:  # Adjust for 90°
                break
        current_state = 'forward'
        #add sleep here
        sock.send(b'done\n')  # <- Notify ESP32
    else:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    print(f"Sensor: {message.strip()} - State: {current_state}")
