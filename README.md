# robotics
Lab 7 of the robotics course in Hanze

---

# Overview

- The **ESP32** acts as the path planner, using Dijkstra's algorithm on a predefined graph to calculate the optimal route.
- The **Webots robot** executes movement commands received from the ESP32 and returns sensor data (ground sensors) for feedback.
- Communication is done via **Wi-Fi sockets**.

---

# Requirements

### Hardware
- ESP32 board (DOIT ESP32 DevKit or similar)
- Wi-Fi access point

# Software
- MicroPython firmware for ESP32
- Thonny IDE (or ampy for file upload)
- Webots simulator (https://cyberbotics.com)
- Python 3.x (on the host machine running Webots)

---

# Dependencies

# On ESP32
- MicroPython standard libraries:
  - `network`, `socket`, `ujson`, `machine`, `time`

# On Host (Webots)
- Webots API (`controller`)
- Python standard libraries:
  - `socket`, `json`, `time`

No external pip packages are required.

---

# Setup Instructions

# 1. ESP32 Setup
- Flash MicroPython to your ESP32 using [official instructions](https://micropython.org/download/esp32/).
- Open `esp32_code/esp32_controller.py` in Thonny and upload it to the ESP32.
- Edit the `ssid` and `password` in the code to match your Wi-Fi network.

# 2. Webots Setup
- Open Webots and create or open your robot simulation.
- Assign `webots_code/webots_controller.py` as the robot controller.
- Ensure the `ESP32_IP` in the Python script matches the actual IP of your ESP32.

---

# Running the System

1. Power on the ESP32 and ensure it connects to Wi-Fi.
2. Start the Webots simulation.
3. The ESP32 will send `"ready"`, then receive start/goal nodes.
4. The robot will move and detect nodes using ground sensors (`gs0`, `gs1`, `gs2`).
5. The ESP32 handles logic, and Webots executes motion commands.

---

# Function Documentation

# `esp32_controller_version_1.py`
- `dijkstra(graph, start, goal)`: Calculates the shortest path using Dijkstra's algorithm.
- `get_turn(current_node, next_node)`: Computes turn direction between path nodes.
- Main loop handles sensor decoding, node detection, and motion decisions.

# `webots_controller_version_1.py`
- Main state machine:
  - `forward`, `turn_left`, `turn_right`, `forward_bit`, `stop`
- Sends ground sensor values to ESP32 and executes movement commands.

---

# Reproducibility Tips

- Set breakpoints or `print()` in both ESP32 and Webots code to debug synchronization.
- Always reset simulation before rerunning to avoid duplicate socket connections.
- Tune sensor thresholds and delay values in `webots_controller.py` for smoother behavior.

---

# License

Educational use only – Robotics Course, Hanze University of Applied Sciences
