# Situational Awareness and Gesture Control for Smart Lights

Project for ECE M202A Embedded Systems.

## Abstract

Smart lights often feature smartphone or voice control, scheduling, and color and brightness selection, but lack any situational awareness capabilities that react to the environment. Additionally, users cannot change these settings without a smartphone or smart speaker, which may not be practical in all situations. The smart light switch proposed here aims to address these issues by increasing automation through situational awareness features such as adaptive brightness and person identification to automatically set lights according to environmental conditions and personal preferences. Additionally, manual control will be possible without other smart devices through advanced gesture control powered by embedded neural networks.

## Project Details

The smart light switch is based on the Arduino Nano 33 BLE Sense and uses the integrated APDS-9960 sensor for gesture control and brightness detection, and the integrated BLE transceiver in the nRF52840 MCU for person identification. Externally, a HC-SR501 PIR motion sensor is used for motion detection and an ESP8266 is used for controlling the WiFi light bulbs. Although tightly integrated with the Arduino environment, underlying Mbed OS APIs are used when possible.

The focus of this project is the advanced gesture control feature. Existing gesture control systems either use cameras and complex neural networks (which require high computational power), or extremely simple hardware and algorithms which can only differentiate between a few different gestures. The Arduino Nano 33 BLE Sense provides a simple solution with the APDS-9960 sensor and can only differentiate between four directions with existing libraries. Using TinyML, we hope to expand the capabilities of the APDS-9960 gesture control system so it can detect simple hand signs as well as additional directions of gestures to allow for more complex commands in the computationally constrained environment.

## Project Timeline

Weeks 1-2: Class begins, order Arduino Nano 33 BLE Sense, explore basic functionality.

Weeks 3-4: Project requirements and past projects presented in class, finalize teams and project plan. Explore basic motion, brightness and gesture sensing.

Weeks 5-6: Add WiFi support with an ESP8266. Integrate WiFi light control with basic motion, brightness and gesture sensing.

Weeks 7-8: Project video on preliminary design/results. Add personal profiles and smartphone detection. Begin working on advanced gesture control.

Weeks 9-10: Deep dive into advanced gesture control with neural networks.

Week 11: Final project video.
