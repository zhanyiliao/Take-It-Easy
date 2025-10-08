# Autonomous Path-Finding Robot Control

This repository contains the main control program for an autonomous robot that navigates a predefined environment using pathfinding algorithms.

## Overview

The program performs the following tasks:

- Loads a map image representing the environment.
- Receives the current position of the robot via a tracking device.
- Computes an optimal path to the destination using a pathfinding algorithm.
- Converts the path into wheel control signals and sends them to the robot.
- Publishes and subscribes to MQTT topics for remote monitoring and control.

This project is primarily intended for educational and research purposes.

## Features

- Real-time path planning and navigation.
- Integration with PCA motor controller and a locator device.
- MQTT-based remote control and monitoring.
- Threaded architecture to separate control and communication.

## Requirements

- Python 3.x
- Required packages:
  ```bash
  pip install numpy scipy pillow paho-mqtt
  ```
- Access to the hardware devices (PCA controller and locator).

## Usage

1. Configure MQTT broker settings:
  ```python
  MQTT_HOST = 'your_broker_ip'
  SUBSCRIBER_TOPIC = 'your/subscriber/topic'
  PUBLISHER_TOPIC = 'your/publisher/topic'
  ```
2. Run the main program:
  ```bash
  python main.py
  ```

The robot will listen to commands from the subscriber topic and execute navigation tasks.

## Notes

- This repository focuses on the control logic. Physical hardware is required for full functionality.
- The `env_lib_2.png` file represents the environment map and should be placed in the project directory.
