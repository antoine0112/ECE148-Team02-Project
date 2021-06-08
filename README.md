# ECE148-Team02-Project
![ECE148-Team02-Car](https://github.com/antoine0112/ECE148-Team02-Project/blob/main/Mechanical/Team2Car.jpg)

As part of ECE/MAE148, Team02 developed an ESP32 vehicle control system, with robust emergency stop features and an expandable webserver-based remote control.
Based on the DonkeyCar's electrical framework, the use of an ESP32 replaces the PCA9685 motor driver, as well as the remote Estop relay board. 
Usage of the ESP32 provides multiple benefits:

- Using a standard USB cable to both power and communicate with serial between the Jetson and ESP32 simplifies wiring. 
- Wifi access point capabilities of the ESP32 enable long range, multi-device access.
- The ESP32's dual core allows for an always-on webserver on one core, with a watchdog monitored motor driver on the other, providing high-speed and robust processing.

Code for flashing the ESP32 can be found under nodeMCU-32S-DualCore/src/main.cpp .
We also recommend using Platform.io for developing on the ESP32, which helps in flashing the webserver Javascript code to the ESP32. 
The Javascript code is found under nodeMCU-32S-DualCore/data/ . 
Configuring a new ESP32 device is simple, just change the Wifi SSID and password parameters.

An example for reading and sending JSON strings through serial can be found under scripts. 
These existing scripts subscribe to ROS /throttle and /steering topics, packaging and sending them over Serial.

This repository is set up to be easily git-cloned as a ROS package. 
Simply create a new package and clone the contents of this repository into the package folder, making sure to edit the package name as necessary.

Also included under mechanical/ are CAD files for camera mounts, base plates, and protective walls which we used on our car. Feel free to alter and use these files.
