# 3D Printed ROS SLAM Robot
A **3D printed robot** that runs on the **Robot Operating System (ROS 2)** and uses a LiDAR to autonomously navigate around and map a room (**Simultaneous Localization And Mapping**).



https://github.com/pliam1105/3D-Printed-ROS-SLAM-Robot/assets/34167782/ba77c825-f32c-44ad-a219-17dad1da1876



The idea for the project was inspired by James Bruton's [ROS SLAM robot](https://www.youtube.com/watch?v=q1u_cC-5Sac), hence the similarity in some parts of the chassis design and electronics (but everything was designed and built from scratch).

## 3D Printed Chassis

For the chassis I decided to learn about CAD design and 3D printing and make it from scratch. Therefore, I learned how to use **Fusion 360** and designed the required components part-by-part, starting with the motor mounts, next the gears and wheels connected with the drive part, and then connecting them and adding the top platform for the LiDAR and electronics.

<table><tr><td align="center"><img src="MEDIA/CAD Fusion 360.png" width="75%"></td></tr><table>

Then I **3D printed** all the parts in my Ender 3 and assembled them with 2 aluminum extrusions, various screws, a rod connecting the wheels, and a couple of ball casters. I also installed all the electronics listed in the next section.

<table><tr><td align="center"><img src="MEDIA/LiDAR Robot.jpg" width="75%"></td></tr><table>

## Hardware & Connections

The electronics components that I used are the following:

* Two **planetary gear 12V DC motors** (gear ratio $19.2:1$) and two **BTS7960 motor drivers** to control them with the Arduino.
* Two **incremental rotary encoders** for odometry data and controlling the motor speed
* An **RPLiDAR A1** for gathering distance measurements in a $360\degree$ range around the robot
* An **Arduino Mega 2560** to control the motors and record the encoder measurements
* An **NVIDIA Jetson Nano**, working on Ubuntu 20.04 and having ROS 2 Foxy installed, to manage the nodes communicating with the Arduino via Serial for motor control and odometry data transmission

To power the robot, I use:

* A **12V 3S Li-PO battery** that directly powers the motors
* An **LM2596 adjustable step down module**, set to 5V to power the encoders
* A **power bank** (or battery charger) for the Jetson Nano
* The Arduino is powered directly via USB (which also works as Serial) by the Jetson Nano

The connections between the components are depicted in the below Fritzing diagram.

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Circuit & Connections.png" width="75%"></td></tr><table>

## Arduino Motor Control with PID ([code](Arduino%20Codes/ROSSerial/ROSSerial.ino))

In order to control the motors, I used an Arduino that transfers the velocity commands (given as linear and angular velocity components) from the Jetson Nano and returns the wheels' speeds at any moment.

### Encoder speed calculation and conversion to $m/s$

The first thing needed was to **measure the encoders' rotation speed and direction**. The encoders have two pins, A and B, which give a `HIGH` or `LOW` signal with a phase difference of $90\degree$. Therefore, to measure the speed of rotation we attach an **interrupt** at pin A that triggers when its signal *rises* from `LOW` to `HIGH` (pulse), and count the number of pulses at a certain time interval. To detect the direction we check if the value of B signal is the same as A when we detect a pulse.

Then, to get the speed in terms of $m/s$ (as required by ROS) instead of $pulses/millisecond$ as computed before, I needed to measure the amount of pulses that account to a meter of robot displacement. To do that I put a measuring tape on the ground, moved the robot 1 meter, and noted the number of pulses on the encoders (although later it turned out to not be accurate so I ran the PID controller described below on a certain speed for a specific time and calculated the conversion rate again based on the distance covered).

<table>
    <tr>
        <td align="center"><img src="MEDIA/Encoder Calibration 2.jpg" width="100%"></td>
        <td align="center"><img src="MEDIA/Encoder Calibration 3.jpg" width="100%"></td>
    </tr>
<table>

### PID controller and differential drive

The next, and most important, thing to be implemented is being able to convert the $(linear, angular)$ velocity commands from ROS to **motor speeds**, and **apply them to the motors**, in order to move the robot in the intended way. We first need to convert the 2 given velocity components to the wheel speeds (in $m/s$), that is, implement **differential drive**, which is done using the formulae:

$$
left\ wheel\ speed = linear_X - angular_Z \cdot wheel\ separation / 2
$$

$$
right\ wheel\ speed = linear_X + angular_Z \cdot wheel\ separation / 2
$$

To make the motors move in those specific speeds and mitigate the effects of different motor powers, asymmetrical forces, slippage, and various others, we use a **PID Controller**, which is a *closed loop controller* that gets the encoder measured speeds (*input*) and changes the motors' PWM duty cycle (*output*), using the motor drivers, with the goal of achieving certain speeds. I used the `PID_v1` Arduino library, and tuned the $KP$, $KI$, $KD$ parameters via trial-and-error, finally achieving fast convergence to the intended speed with great accuracy.

## ROS Motor Control Node ([code](ROS%20Codes/arduino_serial/))

Then, I needed to get ROS working with the Arduino and be able to send velocity commands via the `cmd_vel` topic, and receive odometry data via the `odom` one, also publishing the transform `odom` $\rightarrow$ `base_link` (the position and orientation of the robot estimated by the encoder measurements), for which I will explain more in the [SLAM](#slam-node) section.

On the Jetson Nano side, I installed ROS 2 Foxy, working with Ubuntu 20.04, and implemented a package from scratch (`arduino_serial`) that contains the node (with the same name) that will undertake the above functionality.

### Arduino - Jetson Nano Serial Communication

The only feasible way to transmit the required data from/to the Arduino, since the `rosserial` library only works on ROS 1 and `micro-ROS` doesn't work on older Arduino boards, was to use serial communication between Jetson Nano and Arduino. To that end, I used the [`SerialTransfer`](https://github.com/PowerBroker2/SerialTransfer) library and its Python counterpart, [`pySerialTransfer`](https://github.com/PowerBroker2/pySerialTransfer), to send and receive packets of data.

At any moment, I wanted to send velocity commands in the form described [above](#pid-controller-and-differential-drive), but also receive the speed of each wheel, measured by the encoders. That required synchronized data transfer between Jetson Nano and Arduino. When initializing the ROS node and in the Arduino `setup()` function, I made a *handshake* between those two, that is, send a request every 1 second from the Jetson Nano and wait until Arduino responds. Then, at every cycle of the ROS node and the Arduino (which usually have phase difference):
* In the Jetson Nano side, I first send the velocity commands, then wait to receive the encoder speeds from the Arduino
* In the Arduino side, I first wait to receive the velocity commands, then send the encoder speeds

To do that, I needed to send data even when there weren't any new commands or encoder speeds, so I just resent the latest values available.

### Velocity Commands

As mentioned above, the `arduino_serial` node subscribes to the `/cmd_vel` topic to get messages of type `Twist` ,containing $angular\ x,y,z$ and $linear\ x,y,z$ components, from which we only care about the $linear_x$ and $angular_z$ ones, which it then forwards to the Arduino. I tested that functionality using the `teleop_twist_keyboard` package and the respective node.

### Odometry feedback and pose estimation

The navigation nodes used in the [next section](#simultaneous-localization-and-mapping-code) require the publishing of `Odometry` messages, containing the estimated pose and velocity of the robot, as well as the transform `odom` $\rightarrow$ `base_link`, that is, the estimated pose frame (`base_link`) with respect to a fixed frame (`odom`).

I compute the $(linear, angular)$ velocity components using the following formulae:

$$
linear_x = (left\ wheel\ speed + right\ wheel\ speed) / 2
$$

$$
angular_z = \frac{right\ wheel\ speed - left\ wheel\ speed}{wheel\ separation}
$$

I also time-integrate the wheel speeds to get the pose estimation using the following formulae (applied at each time-step with new encoder data):

$$
\text{Angle displacement:}\ d\theta = angular_z \cdot dt
$$

$$
\text{Distance left wheel traveled:}\ d_{left} = left\ wheel\ speed \cdot dt
$$

$$
\text{Distance right wheel traveled:}\ d_{right} = right\ wheel\ speed \cdot dt
$$

$$
\text{Distance point of reference (center) traveled:}\ d_{center} = (d_{left} + d_{right})/2
$$

$$
\text{$x$ displacement:}\ d_x = d_{center} \cdot \cos{(\theta + d\theta/2)}
$$

$$
\text{$y$ displacement:}\ d_y = d_{center} \cdot \sin{(\theta + d\theta/2)}
$$

$$
\text{New $x$ coordinate:}\ x' = x + dx
$$

$$
\text{New $y$ coordinate:}\ y' = y + dy
$$

$$
\text{New $\theta$ orientation:}\ \theta' = \theta + d\theta
$$

I tested the odometry pose estimation by moving the robot around with `teleop_twist_keyboard` around the room and checking the drift of LiDAR data with respect to a fixed coordinate frame (`odom`), by visualizing it with RViz 2.

<table><tr><td align="center"><img src="MEDIA/Teleop & RViz LiDAR.png" width="75%"></td></tr><table><br>

## Simultaneous Localization and Mapping ([code](ROS%20Codes/slam_bot/))

The next step, after being able to control the robot's velocity and get a pose estimation based on its encoders, is to incorporate LiDAR data and implement **Simultaneous Localization and Mapping (SLAM)**, and also **navigate** around the room autonomously.

### LiDAR Node

To convert the LiDAR data, which are transmitted via USB to the Jetson Nano, to a ROS-compatible format (`LaserScan` messages), I used the [RPLiDAR ROS](https://github.com/allenh1/rplidar_ros) library, which launches a node that publishes to the `/scan` topic, which I could visualize in RViz 2 as shown below.

<table><tr><td align="center"><img src="MEDIA/RViz LiDAR.png" width="75%"></td></tr><table><br>

### Robot & Joint State Publishers

ROS coordinates are based on certain *frames*, that is coordinate frames that are related with each other using *transforms* (which can be published and computed using `tf2`), that are all part of the *TF tree*. In terms of the robot, these frames are described using *links*, which are connected with each other via child-parent relationships and defined using translations.

Therefore, a robot is described by defining all these links and relations in a URDF file (an XML file). This description is published in the TF tree using a *Robot State Publisher* (using the `robot_state_publisher` package and respective node).

Also, since there are joints with variable orientation (or other types of movement), their states are published using a *Joint State Publisher* (using the `joint_state_publisher` package and node and their GUI counterparts that enable changing the states using sliders).

<table><tr><td align="center"><img src="MEDIA/SLAM Bot URDF Visual.png" width="75%"></td></tr><table><br>

### SLAM Node

This is the point where all the above aspects start connecting with each other. The fundamental part of an autonomous robot is being able to **map an unknown area and localize itself in it**; this is the essence of **SLAM**. To implement that functionality, I used the `slam_toolbox` package, specifically the `async_slam_toolbox_node` node, which receives the odometry and sensory data published and creates a map, which it aligns with the robot's coordinate frame to predict a more accurate location estimation.

That's where the coordinate transforms mentioned in the [Motor Control Node](#ros-motor-control-node-code) section come handy. The core frames used in localization and mapping using ROS are: `map`, `odom`, and `base_link` (and `base_laser` or `laser` representing LiDAR, but this is usually computed with respect to `base_link`).

* `map` is the frame that represented the real-world environment of the robot, which is supposed to be stationary
* `odom` is the frame that stays fixed with respect to the starting position of the robot, and is used to depict the robot's odometry location estimate
* `base_link` is the frame that represents the robot's position

The `odom` $\rightarrow$ `base_link` transform is computed as described in the [odometry](#odometry-feedback-and-pose-estimation) section, using the encoder data.

The `map` $\rightarrow$ `odom` transform is computed by the localization node, in this case one from the `slam_toolbox` package, and its purpose is to align the robot's estimate of its position with the map of its environment using its sensory data.

The result of that functionality is visualized as shown below, in RViz 2.

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Mapping & RViz.png" width="75%"></td></tr><table><br>

### Navigation Nodes

After locating itself in the environment, the robot's purpose is to complete tasks in it, and this involves moving to specific locations on its own. This is the navigation part of the project, which is implemented using the `navigation2` package (and the `nav2_bringup` to launch all the required nodes at once).

It takes the map computed from `slam_toolbox` and creates a *costmap* based on it, that is, a map representing the ability to traverse each part of the map, with relation to the occupied areas (which are also *inflated* by a radius, accounting for the robot's size).

Then, a *planner* takes that costmap and the end goal given by the user (via RViz or a service call), and computes, for some time ahead, the optimal path to achieve that goal. This updates at specific intervals, taking into account the new state of the robot and updated map.

Finally, a *controller* takes the path computed by the planner and publishes the velocity commands to follow it, which then the Arduino receives and applies motor speeds accordingly. The controller also updates at specific intervals, integrating the newest path and robot state received.

There are also some *backup nodes* that are activated in certain instances to recover the robot's location and find a new way to accomplish the goal (all this structure works using decision trees).

A visualization of the state of the robot, the map, and the path computed in a certain instance, in RViz 2 is provided below.

<table><tr><td align="center"><img src="MEDIA/SLAM Bot Navigation RViz.jpg" width=75%"></td></tr><table><br>

## Final Result

The final result is a robot that is able to **receive end goal commands and navigate to them on its own**, able to adapt to unknown environments and unexpected instances effectively. A snippet of the robot working in real life is given below.

<table><tr><td align="center"><img src="MEDIA/SLAM Navigation RViz & Camera GIF.gif" width="100%"></td></tr><table><br>
