import network
import socket
import ujson
from machine import Pin
from time import sleep

# --- Connect to Wi-Fi ---
ssid = 'TP-Link_CA3E' # For HARd router
password = '52601422'

#ssid = 'HANZE-ZP11'
#password = 'sqr274YzW6'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

print("Connecting to Wi-Fi...")
while not wlan.isconnected():
    sleep(0.5)
print("Connected, IP:", wlan.ifconfig()[0])

# --- Setup TCP socket server ---
server_socket = socket.socket()
server_socket.bind(('', 8888))  # Listen on all interfaces, port 8888
server_socket.listen(1)
print("Waiting for Webots connection...")

conn, addr = server_socket.accept()
print("Connected to Webots:", addr)

# --- Button setup ---
button = Pin(34, Pin.IN, Pin.PULL_DOWN)

# --- Graph definition (same as before, paste full one) ---
graph = {
    # Top row
    'A1': [('S1', 1, 2)],
    'A2': [('S2', 1, 2)],
    'A3': [('S3', 1, 2)],
    'A4': [('S4', 1, 2)],
    
    # Entry lines
    'S1': [('A1', 1, 0), ('S2', 1, 1), ('F', 5, 2)],
    'S2': [('A2', 1, 0), ('S3', 1, 1), ('S1', 1, 3)],
    'S3': [('A3', 1, 0), ('S4', 1, 1), ('S2', 1, 3)],
    'S4': [('A4', 1, 0), ('L', 2, 1), ('S3', 1, 3)],
    
    #Right quarter
    'L': [('K', 5, 1), ('M', 3, 2), ('S4', 2, 3)],
    'K': [('J', 3, 2), ('L', 5, 3)],
    'M': [('L', 3, 0), ('J', 5, 1), ('H', 2, 2)],
    'J': [('K', 3, 0), ('I', 2, 2), ('M', 5, 3)],

    # Middle
    'F': [('S1', 5, 0), ('H', 5, 1), ('E', 2, 2)],
    'H': [('M', 2, 0), ('I', 5, 1), ('O', 2, 2), ('F', 5, 3)],
    'I': [('J', 2, 0), ('G4', 5, 2), ('H', 5, 3)],
    'E': [('F', 2, 0), ('O', 5, 1), ('D', 3, 2)],
    'O': [('H', 2, 0), ('C', 3, 2), ('E', 5, 3)],
    
    #Left quarter
    'D': [('E', 3, 0), ('C', 5, 1)],
    'C': [('O', 3, 0), ('G1', 2, 1), ('D', 5, 3)],

    # Entry lines
    'G1': [('G2', 1, 1), ('B1', 1, 2), ('C', 2, 3)],
    'G2': [('G3', 1, 1), ('B2', 1, 2), ('G1', 1, 3)],
    'G3': [('G4', 1, 1), ('B3', 1, 2), ('G2', 1, 3)],
    'G4': [('I', 5, 0), ('B4', 1, 2), ('G3', 1, 3)],
    
    #Bottom row
    'B1': [('G1', 1, 0)],
    'B2': [('G2', 1, 0)],
    'B3': [('G3', 1, 0)],
    'B4': [('G4', 1, 0)],
}
node_detected = False
webots_not_busy = True

def dijkstra(graph, start, goal):
    shortest_distance = {node: float('inf') for node in graph}
    shortest_distance[start] = 0
    predecessor = {}
    unvisited = set(graph.keys())
    while unvisited:
        current = min((node for node in unvisited), key=lambda n: shortest_distance[n])
        if current == goal:
            break
        unvisited.remove(current)
        for neighbor, cost, _ in graph[current]:
            distance = shortest_distance[current] + cost
            if distance < shortest_distance[neighbor]:
                shortest_distance[neighbor] = distance
                predecessor[neighbor] = current
    path = []
    current = goal
    while current != start:
        if current not in predecessor:
            return []
        path.insert(0, current)
        current = predecessor[current]
    path.insert(0, start)
    return path

# Get direction to neighbour node from Dijkstra path
def get_turn(current_node, next_node):
    for neighbor, cost, direction in graph[current_node]:
        if neighbor == next_node:
            return direction
        
# --- Wait for button ---
print("Press button to start")
while not button():
    sleep(0.1)

sleep(0.6)
conn.send(b'ready\n')

# --- Wait for start/goal from Webots ---
while True:
    msg = conn.recv(1024).decode().strip()
    try:
        data = ujson.loads(msg)
        path = dijkstra(graph, data['start'], data['goal'])
        conn.send((ujson.dumps({'path': path}) + '\n').encode())
        print("[PATH FOUND]", path)
        break
    except:
        continue

# --- Movement logic ---
path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]
heading = get_turn(current_node, next_node)
# Force init heading
print(f"[INIT] Initial heading from {current_node} to {next_node}: {heading}")

while True:
    msg = conn.recv(1024).decode().strip()
    # --- Handle "done" confirmation from Webots ---
    if msg == 'done':
        webots_not_busy = True
        print("[SYNC] Webots finished action")
        continue
      # --- Handle sensor data: only proceed if length is exactly 3 ---
    if len(msg) != 3:
        continue
    
    left, center, right = msg[0] == '1', msg[1] == '1', msg[2] == '1'

    if left and center and right and webots_not_busy:  # Detected node (111 = all black)
        #Node detection
        if node_detected == False:  # Only trigger once per node
            print("Node is detected")
            node_detected = True
            conn.send(b'stop\n')
            #sleep(1)
            sleep(0.1)
            
            # Check if goal_node was reached
            if path_index + 1 >= len(path):
                conn.send(b'stop\n')
                print("The goal was reached")
                continue
            
            #Changing variables to new current node and next node
            path_index += 1
            current_node = path[path_index]
            next_node = path[path_index + 1]
            heading = get_turn(current_node, next_node)

            print(f"[Move] From: {current_node}, To: {next_node}, Heading: {heading}")
            # Webots should know that heading was updated so it is new node and should go forwad a bit to not have 111 on sensosors
            if heading == 0:
                conn.send(b'forward_bit\n')
            elif heading == 1:
                conn.send(b'turn_right\n')
            elif heading == 3:
                conn.send(b'turn_left\n')
            heading = 0
            webots_not_busy = False
            # Should wait for Webots finishing turning and going forward a bit that sensor get not 111
    else:
        node_detected = False  # Reset when node is left
        conn.send(b'forward\n')


