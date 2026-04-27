# Conveyor-Color-Sorting-System

This is the code for my Arduino-based conveyor sorting system using a TCS3200 color sensor. Objects are classifed by RGB values and sorted using time-based actuation.

System Overview:
  - Conveyor moves objects past sensor
  - Sensor reads RGB values
  - Code classifies object (red, yellow, blue, light, dark)
  - Actuators sort object into correct output bin

Main Hardware:
  - Arduino Mega 2560
  - TCS3200 sensor
  - DC motors for the conveyor and pushers
  - Servo motor for the light/dark servo
  - Motor drivers and the power supplies

Software:
  - Written in C++
  - Uses RGB sampling and calibration
  - Time-based scheduling for actuation
  - Simple state machine control

Installation:
  - Open code in Arduino IDE
  - Connect Arduino Mega
  - Upload Code

How to run:
  - Power the system
  - Place the different color objects on the conveyor
  - The system then detects, classifies, and sorts automatically

Known limitations:
  - Depends on constant and consistent belt speed
  - Sensitive to lighting conditions
  - No position feedback (open-loop)

Future Work:
  - Add encoder feedback
  - Implement closed-loop control

    
