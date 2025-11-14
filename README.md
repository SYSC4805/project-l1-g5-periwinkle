# Roadmap of Code Repository for L1-G5's Autonomous Snowplower
### Group L1-G5: Periwinkle
### SYSC 4310
### Team Members
- Hamnah Qureshi, 101225634
- Alvan Chaudhury, 101272922
- Noah Oatley, 101189707

### TA: Ehsan Adel Rastkhiz

## Overview of Autonomous Snowplower 

The autonomous snowploer is robot that is capable of efficiently clearing simulated snow (lightweight wooden blocks) from a controlled indoor arena while navigating static and moving obstacles. The robot can complete this task within the 5-min limit by optimizing path planning and obstacle avoidance using sensor inputs and robust control algorithms. 


## Instructions on How to Start Up the System

### Requirements
- Arduino Due 
- Robot Chassis
- 5 AA batteries
- Motor Board connected to the encoders and wheels of the robot
- The following sensors connected to the chassis 
    - Line Follower in front and rear of the chassis
    - Obstacle detection IR sensor connected to the front bottom of the chassis
    - Ultrasonic sensor connected to front top of the chassis 
- MinIMU-9 v5 Gyro, Accelerometer, and Compass
- Arduino IDE installed 
- Download https://github.com/bdmihai/DueFreeRTOS to Arduino's libraries folder which is most likley in `~/Documents/Arduino/libraries`

### To Run The Code
- Connect your computer to the programming port of the Arduino Due 
- Open `ObstacleDetectingRTOS.ino` file in the Arduino IDE
- Make sure the IDE detects the correct COM port for the Arduino Due 
- Click the upload button, the IDE will compile and flash the program to the board
- Disconnect the USB cable from the programming port 
- Insert 5 AA batteries in the robot's battery holder
- Turn on the switch at top of robot chassis
- The robot should start driving automatically

