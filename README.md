# Pan-Tilt Air Defense System
Overview

The Pan-Tilt Air Defense System is a computer vision-based project designed to track and follow targets using facial recognition. This system utilizes a camera and an Arduino-based pan-tilt mechanism to autonomously track and center a detected face on the screen.

It integrates a PID controller to ensure smooth and accurate movement of the pan-tilt system and employs the OpenCV library for real-time facial recognition. The system continuously adjusts the motors to keep the detected face centered while avoiding unnecessary movements using a deadband approach.

Features

Real-time face detection and tracking using Haar Cascade Classifier.

PID controller for precise motor control.

Arduino communication over serial port for motor commands.

Adjustable PID tuning parameters for stability and responsiveness.

Efficient handling of motor commands via a queue, ensuring smooth operation.

Requirements

Python 3.x

OpenCV (cv2) library for computer vision

pySerial library for communication with Arduino

Haar Cascade XML file for face detection (haarcascade_frontalface_default.xml)

Installation

Clone the repository:

git clone https://github.com/yourusername/pan-tilt-air-defense-system.git
cd pan-tilt-air-defense-system


Install dependencies:

pip install opencv-python pyserial


Download the Haar Cascade Classifier:

Download the file haarcascade_frontalface_default.xml from OpenCV GitHub repository
 and place it in the same directory as the script.

Setup Arduino:

Connect an Arduino-based pan-tilt motor system. Make sure the correct serial port is configured in the script (e.g., /dev/ttyUSB0 for Linux or COMx for Windows).

Usage

Run the script:

python pan_tilt_tracking.py


Exit the program:

To stop the tracking, press the ESC key.

System Configuration
Camera

The system uses the default webcam (device 0) with a resolution of 640x480 pixels.

Arduino Communication

The Arduino expects commands in the format of two comma-separated values (for X and Y motor angles) sent over the serial port. These are calculated using the PID controller based on the angular error between the detected face and the center of the camera's view.

PID Controller

The PID controller is configured with the following initial values:

kp = 1.2

ki = 0.02

kd = 0.10

These values can be adjusted for different performance characteristics.
