from pySerialTransfer import pySerialTransfer as txfer
import time

class data_struct(object):
    def __init__(self, vel_left=0.0, vel_right=0.0, stop=1, angle1 = 0.0, angle2 = 10.0, angle3 = 50.0, angle4 = 25.0):
        self.vel_left = vel_left # decimal (float)
        self.vel_right = vel_right # decimal (float)
        self.stop = stop # integer 0 or 1
        self.angle1 = angle1 #decimal (float)
        self.angle2 = angle2 #decimal (float)
        self.angle3 = angle3 #decimal (float)
        self.angle4 = angle4 #decimal (float)
    def __repr__(self):
        return "left speed: "+str(self.vel_left)+" right speed: "+str(self.vel_right)+" stop: "+str(self.stop)+" servos: "+repr([self.angle1, self.angle2, self.angle3, self.angle4])

# Arduino serial communication class
class ArduinoSerial:
    def __init__(self):
        # data setup
        self.send_data = data_struct()
        self.real_data = data_struct()
        # self.last_vel_update_time = time.time()
    
    def arduino_handshake(self):
        # initiate connection
        self.link = txfer.SerialTransfer('ttyACM0')
        self.link.open()
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

    def print_vals(self):
        print("Send data:", self.send_data)
        print("Real data:", self.real_data)

    def get_vals(self):
        return self.send_data, self.real_data
    
    # def arduino_update(self):
    #     # check if idle for long and stop
    #     vel_time_idle = time.time() - self.last_vel_update_time
    #     # print("Time Idle: {0}".format(time_idle_sec))
    #     if(vel_time_idle > 5):
    #         self.stop()
    #     self.arduino_ensure_get_send()
    
    def arduino_stop(self):
        self.send_data.vel_left = 0.0
        self.send_data.vel_right = 0.0
        self.send_data.stop = 1
        self.arduino_ensure_get_send()

    # for some fucking reason it only works after sending the same command twice, IF IT AIN'T BROKE DON'T FIX IT
    def arduino_ensure_get_send(self):
        self.arduino_get_send()
        self.arduino_get_send()
        print("Arduino get-send succesful!")
        self.print_vals()
    
    def arduino_get_send(self):
        send_packet_size = 0 
        send_packet_size = self.link.tx_obj(self.send_data.vel_left, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.vel_right, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.stop, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angle1, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angle2, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angle3, start_pos=send_packet_size)
        send_packet_size = self.link.tx_obj(self.send_data.angle4, start_pos=send_packet_size)
        self.link.send(send_packet_size)
        
        # receiving encoder speeds
        while not self.link.available():
            pass

        receive_packet_size = 0
        self.real_data.vel_left = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_data.vel_right = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_data.stop = self.link.rx_obj(obj_type='i', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['i']
        self.real_data.angle1 = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_data.angle2 = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_data.angle3 = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.real_data.angle4 = self.link.rx_obj(obj_type='f', start_pos=receive_packet_size)
        receive_packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']

    def arduino_update_motors(self, left_speed, right_speed):
        self.send_data.vel_left = float(left_speed)
        self.send_data.vel_right = float(right_speed)
        if(self.send_data.vel_left == 0 and self.send_data.vel_right == 0):
            self.send_data.stop = 1
        else:
            self.send_data.stop = 0
        # self.last_vel_update_time = time.time()
        self.arduino_ensure_get_send()


    def arduino_update_servos(self, angle1, angle2, angle3, angle4):
        self.send_data.angle1 = float(angle1)
        self.send_data.angle2 = float(angle2)
        self.send_data.angle3 = float(angle3)
        self.send_data.angle4 = float(angle4)
        self.arduino_ensure_get_send()
    
    def arduino_read_motor_speeds(self):
        self.arduino_ensure_get_send()
        return self.real_data.vel_left, self.real_data.vel_right