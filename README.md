# Robotics Simulation Project

This repository contains simulation code for a line-following and path-navigating robot using Webots. The primary controller script is located in [`webots_code/webots_controller_v4_final.py`](webots_code/webots_controller_v4_final.py). The system is designed to work in conjunction with an ESP32 microcontroller over UART or Wi-Fi to receive navigation commands and sensor data.

## Features

- **Webots Simulation**: Uses the Webots simulator for realistic robot physics and environment interaction.
- **Line-Following**: Robot can follow black lines on a white background using ground sensors.
- **Node-Based Navigation**: Robot detects nodes (intersections) and navigates using predefined paths.
- **ESP32 Communication**: Path planning is offloaded to an ESP32 running Dijkstra's algorithm; communication is done via serial or Wi-Fi socket.
- **Command Handling**: Receives instructions such as `forward_bit`, `turn_left`, `turn_right`, and `stop`.

## How It Works

1. **Robot Behavior**:
   - Follows a line using three ground sensors.
   - Detects nodes by reading all sensors as active (111).
   - Awaits commands from ESP32 to determine next movement.

2. **ESP32 Role**:
   - Calculates optimal path using Dijkstra's algorithm.
   - Sends movement commands via UART or TCP to Webots.

3. **Movement Commands**:
   - `forward_bit`: Move forward a segment.
   - `turn_left` / `turn_right`: Rotate and re-align to new direction.
   - `stop`: Stop at the goal.

## Requirements

- Webots (tested with R2023b and later)
- Python controller environment enabled in Webots
- ESP32 (MicroPython or Arduino-based)
- PC and ESP32 on same network (for Wi-Fi mode)

## Usage

1. Open your Webots project world.
2. Assign the robot the controller `webots_controller_v4_final`.
3. Start the simulation.
4. Ensure ESP32 is running its path-planning script and connected.


