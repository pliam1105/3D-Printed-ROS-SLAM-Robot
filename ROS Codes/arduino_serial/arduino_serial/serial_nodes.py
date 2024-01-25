import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time

from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String

import tf_transformations
from tf2_ros import TransformBroadcaster

class vel_struct(object):
    def __init__(self, linear_x=0.0, angular_z=0.0, stop=1):
        self.linear_x = linear_x # decimal (float)
        self.angular_z = angular_z # decimal (float)
        self.stop = stop # integer 0 or 1

class real_speed_struct(object):
    def __init__(self, speed_left=0.0, speed_right=0.0):
        self.speed_left = speed_left
        self.speed_right = speed_right

# Arduino serial communication node
class ArduinoSerial(Node):
    def __init__(self):
        super().__init__('arduino_serial')
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_sub # prevent unused variable warning
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_tf_pub = TransformBroadcaster(self)
        publish_timer_period = 0.5 # seconds
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_timer_callback)
        get_send_timer_period = 0.01 # seconds
        self.get_send_timer = self.create_timer(get_send_timer_period, self.arduino_update)
        # time
        self.last_odom_time = self.last_vel_time = self.get_clock().now()
        # odometry data
        self.odom_pos_x = self.odom_pos_y = 0.0 # position
        self.odom_linear_x = self.odom_angular_z = 0.0 # velocity
        self.odom_roll = self.odom_pitch = self.odom_yaw = 0.0 # orientation
        self.wheel_separation = 0.25
        # other setup
        self.send_data = vel_struct()
        self.real_speed = real_speed_struct()
        # initiate connection
        self.link = txfer.SerialTransfer('ttyACM0')
        self.link.open()
        self.arduino_handshake()
    
    def arduino_handshake(self):
        while True:
            # send call
            call_packet_size = 0
            call_packet_size = self.link.tx_obj(100, call_packet_size)
            self.link.send(call_packet_size)
            # get string response
            if(self.link.available()):
                ok_packet_size = 0
                ok = self.link.rx_obj(obj_type='i', start_pos=ok_packet_size)# should be 200
                print("Handshake response: {0}".format(ok))
                if(ok == 200):
                    print("Handshake Succesful!")
                    break
            time.sleep(1)

    def update_odometry(self):
        # update odometry data
        self.odom_linear_x = (self.real_speed.speed_left + self.real_speed.speed_right) / 2
        self.odom_angular_z = (self.real_speed.speed_right - self.real_speed.speed_left) / self.wheel_separation
        # print("left: {0}, right: {1}, linear: {2}, angular: {3}".format(self.real_speed.speed_left, self.real_speed.speed_right, self.odom_linear_x, self.odom_angular_z))
        curr_time = self.get_clock().now()
        # compute changes
        dt = (curr_time - self.last_odom_time).nanoseconds / 1e9
        # print("dt: {0}".format(dt))
        dth = self.odom_angular_z * dt
        move_left = self.real_speed.speed_left * dt
        move_right = self.real_speed.speed_right * dt
        move_center = (move_left + move_right) / 2
        dx = move_center * math.cos(self.odom_yaw+dth/2)
        dy = move_center * math.sin(self.odom_yaw+dth/2)
        # apply changes
        self.odom_pos_x += dx
        self.odom_pos_y += dy
        self.odom_yaw += dth
        # update time
        self.last_odom_time = curr_time
    
    def arduino_update(self):
        # check if idle for long and stop
        time_idle = self.get_clock().now() - self.last_vel_time
        time_idle_sec = time_idle.nanoseconds / 1e9
        # print("Time Idle: {0}".format(time_idle_sec))
        if(time_idle_sec > 5):
            self.stop()
        self.arduino_get_send()
        self.update_odometry()       

    def stop(self):
        self.send_data.linear_x = 0.0
        self.send_data.angular_z = 0.0
        self.send_data.stop = 1
    
    def arduino_get_send(self):
        # send commands
        # make packet
        send_packet_size = 0 
        send_packet_size = self.link.tx_obj(self.send_data.linear_x, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angular_z, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.stop, start_pos=send_packet_size)
        # send packet
        self.link.send(send_packet_size)
        # print("New command sent: {0} , {1} , {2}".format(self.send_data.linear_x, self.send_data.angular_z, self.send_data.stop))
        
        # receiving encoder speeds
        while not self.link.available():
            pass
        receive_packet_size = 0
        self.real_speed.speed_left = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_speed.speed_right = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # print("Real speed received: {0}, {1}".format(self.real_speed.speed_left, self.real_speed.speed_right))
    
    def cmd_vel_callback(self, cmd_vel_msg):
        self.get_logger().info("Velocity command heard: {0} , {1}".format(cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))
        self.send_data.linear_x = cmd_vel_msg.linear.x
        self.send_data.angular_z = cmd_vel_msg.angular.z
        if(self.send_data.linear_x == 0 and self.send_data.angular_z == 0):
            self.send_data.stop = True
        else:
            self.send_data.stop = False
        # update idle time
        self.last_vel_time = self.get_clock().now()

    def publish_timer_callback(self):
        odom_msg = Odometry()
        # put data in message
        odom_msg.header.stamp = self.last_odom_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.odom_pos_x
        odom_msg.pose.pose.position.y = self.odom_pos_y
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = tf_transformations.quaternion_from_euler(self.odom_roll, self.odom_pitch, self.odom_yaw)
        odom_msg.twist.twist.linear.x = self.odom_linear_x
        odom_msg.twist.twist.angular.z = self.odom_angular_z
        # publish odometry
        self.odom_pub.publish(odom_msg)
        # create odom->base_link transformation and publish it
        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.last_odom_time.to_msg()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation.x = self.odom_pos_x
        odom_tf.transform.translation.y = self.odom_pos_y
        odom_tf.transform.rotation.x, odom_tf.transform.rotation.y, odom_tf.transform.rotation.z, odom_tf.transform.rotation.w = tf_transformations.quaternion_from_euler(self.odom_roll, self.odom_pitch, self.odom_yaw)
        self.odom_tf_pub.sendTransform(odom_tf)

def main(args=None):
    rclpy.init(args=args)
    
    arduino_serial = ArduinoSerial()
    
    rclpy.spin(arduino_serial)
    
    # shutdown
    rclpy.shutdown()