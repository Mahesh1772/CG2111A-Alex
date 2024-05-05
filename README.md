# Alex to the Rescue

Alex is an autonomous robotic system designed for search and rescue operations in simulated environments. This project aims to create a remotely controlled robot capable of navigating through complex terrains, mapping its surroundings, and detecting potential victims or objects of interest using various sensors and algorithms.

## System Overview

### Core Functionalities

- **Remote Controllability**: Alex can be maneuvered by an operator without direct line of sight, through wireless commands sent over a secure Wi-Fi connection using VNC.
- **Environmental Mapping**: The robot utilizes a LiDAR (Light Detection and Ranging) sensor to create a detailed 3D map of its surroundings, enabling the operator to issue accurate movement instructions and navigate through the environment effectively.
- **Movement and Navigation**: Alex can move forward, backward, turn left, and turn right. Its movement is precisely controlled by providing distance or angle inputs, along with speed percentages. Hall Effect sensors attached to the motors track the rotations, enabling accurate positioning and navigation.

### System Architecture

1. **Raspberry Pi 4**: The central processing unit responsible for communicating with the operator's laptop, processing sensor data, and controlling the Arduino Uno.
2. **Arduino Uno**: Handles low-level operations, such as motor control, sensor data acquisition, and communication with the Raspberry Pi.
3. **LiDAR**: Used for mapping the environment by emitting pulses of light and detecting the reflected signals to calculate distances and create a 3D representation of the surroundings.
4. **Motors and Wheel Encoders**: Two motors with a 48:1 gear ratio drive Alex's movement, while wheel encoders precisely measure the revolutions and distance traveled.
5. **Power System**: The Raspberry Pi is powered by a power bank, while the motors receive power from a separate 5V battery pack.
6. **Ultrasonic Sensors**: Two ultrasonic sensors mounted on the sides of Alex detect obstacles in close proximity, enabling collision avoidance and safe navigation.
7. **Color Sensor**: A color sensor mounted at the front of Alex detects the color of potential victims or objects of interest, aiding in the rescue mission.

## Operation Sequence

1. **Initialization**: Alex establishes a secure TLS (Transport Layer Security) connection between the operator's laptop and the Raspberry Pi, enabling wireless communication and control.
2. **User Command Reception**: The operator issues movement commands (forward, backward, left, right, stop) through the laptop, which are transmitted to the Raspberry Pi and forwarded to the Arduino.
3. **Command Execution**: The Arduino interprets the received commands and controls the motors accordingly, allowing Alex to navigate the environment.
4. **Environment Mapping**: Simultaneously, the LiDAR sensor continuously scans the surroundings, and the collected data is processed using the Hector SLAM (Simultaneous Localization and Mapping) algorithm to generate a live map, which is displayed to the operator.
5. **Victim/Object Detection**: When Alex encounters a potential victim or object of interest, the color sensor is activated, and the detected color is relayed back to the operator through the secure connection.

With its advanced sensor suite, robust communication system, and precise navigation capabilities, Alex is designed to provide a reliable and efficient solution for search and rescue operations in hazardous or inaccessible environments.
