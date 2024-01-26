# 3D Printed ROS SLAM Robot
A 3D printed robot that runs on the Robot Operating System (ROS 2) and uses a LiDAR to autonomously navigate around and map a room (Simultaneous Localization And Mapping).

*insert side-to-side video here via github editor*
## 3D Printed Chassis

<table><tr><td align="center"><img src="MEDIA/CAD Fusion 360.png" width="100%"></td></tr><table><br>

## Hardware & Connections

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Circuit & Connections.png" width="100%"></td></tr><table><br>

## Arduino Motor Control with PID ([code](Arduino%20Codes/ROSSerial/ROSSerial.ino))

## ROS Motor Control Node ([code](ROS%20Codes/arduino_serial/))

### Arduino - Jetson Nano Serial Communication

## Simultaneous Localization and Mapping ([code](ROS%20Codes/slam_bot/))

### LiDAR Node

<table><tr><td align="center"><img src="MEDIA/RViz LiDAR.png" width="100%"></td></tr><table><br>

### Robot State & Joint Publisher

<table><tr><td align="center"><img src="MEDIA/SLAM Bot URDF Visual.png" width="100%"></td></tr><table><br>

### SLAM Node

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Mapping & RViz.png" width="100%"></td></tr><table><br>

### Navigation Nodes

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Navigation RViz.jpg" width="100%"></td></tr><table><br>

## Final Result

<table><tr><td align="center"><img src="MEDIA/SLAM Navigation RViz & Camera GIF.gif" width="100%"></td></tr><table><br>