import rclpy
from rclpy.node import Node
import time

from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        # self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.publisher_ = self.create_publisher(String, 'debug', 10)
        publish_timer_period = 0.5 # seconds
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_timer_callback)
        get_send_timer_period = 0.01 # seconds
        self.get_send_timer = self.create_timer(get_send_timer_period, self.arduino_get_send)

        # other setup
        self.send_data = vel_struct()
        self.real_speed = real_speed_struct()
        # initiate connection
        self.link = txfer.SerialTransfer('ttyACM0')
        self.link.open()
        # handshake
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
    
    def arduino_get_send(self):
        # send commands
        # make packet
        send_packet_size = 0 
        send_packet_size = self.link.tx_obj(self.send_data.linear_x, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angular_z, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.stop, start_pos=send_packet_size)
        # send packet
        self.link.send(send_packet_size)
        print("New command sent: {0} , {1} , {2}".format(self.send_data.linear_x, self.send_data.angular_z, self.send_data.stop))
        
        # receiving encoder speeds
        while not self.link.available():
            pass
        receive_packet_size = 0
        self.real_speed.speed_left = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_speed.speed_right = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        print("Real speed received: {0}, {1}".format(self.real_speed.speed_left, self.real_speed.speed_right))
    
    def listener_callback(self, msg):
        self.get_logger().info("Velocity command heard: {0} , {1}".format(msg.linear.x, msg.angular.z))
        self.send_data.linear_x = msg.linear.x
        self.send_data.angular_z = msg.angular.z
        if(self.send_data.linear_x == 0 and self.send_data.angular_z == 0):
            self.send_data.stop = True
        else:
            self.send_data.stop = False
        self.to_send = True

    def publish_timer_callback(self):
        # publish
        msg = String()
        msg.data = "Real speed: {0}, {1}".format(self.real_speed.speed_left, self.real_speed.speed_right)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    arduino_serial = ArduinoSerial()
    
    rclpy.spin(arduino_serial)
    
    # shutdown
    rclpy.shutdown()