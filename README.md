# Gesture-Controlled RoArm Manipulator

This project implements gesture-based control of the RoArm robotic manipulator using ROS2 and Python.  
The system uses a publisher-subscriber model to interpret hand gestures and send corresponding movement commands to the manipulator.
Hand gestures are captured through a webcam and processed using MediaPipe and OpenCV, with gesture data transmitted over ROS 2 to an ESP32-controlled robotic arm.

🧰 Requirements
Ubuntu 20.04 or 22.04
Python 3.8+
ROS 2 (Foxy or Humble recommended)
MediaPipe
OpenCV
An ESP32 microcontroller
RoArm-M2-S manipulator
Waveshare ST3215 Servo Driver
Webcam (e.g., HP Wide Vision 720p or Spedal HFR 1080p)

🚀 How to Run
Step 1: Set up ROS 2 environment
Source your ROS 2 setup in each terminal
Step 2: Run Gesture Publisher Node
This node detects hand landmarks using a webcam and publishes normalized positions.
Step 3: Run Gesture Subscriber Node
This node listens for gesture data, parses it, and sends movement commands to the ESP32 over HTTP.
Before running, replace the self.ip_addr value in gesture_subscriber.py with the IP address of your ESP32.

🎯 How It Works
The gesture_publisher.py script uses MediaPipe Hands to track the user's hand in real time.
It maps the position and distance of key points (like fingertips) to control the robot’s X, Y, Z coordinates and T (tool).
The gesture_subscriber.py receives this data and sends it to ESP32 through HTTP in JSON format.
The ESP32 parses the command and sends appropriate PWM signals to the ST3215 servo driver to control the robotic arm.

📸 Example Gestures
✌️ Peace sign → turns LED ON
Closed fingers → turns LED OFF
Forward/Backward, Up/Down, Left/Right hand motion → moves the manipulator in real time

🛠️ Troubleshooting
Make sure your webcam is working and accessible by OpenCV.
Ensure the ESP32 is reachable over the local network (check IP address).
Confirm the ST3215 driver is properly powered and wired to the servos and ESP32.

