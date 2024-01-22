from time import sleep
from pySerialTransfer import pySerialTransfer as txfer

unsent = False

class vel_struct(object):
    def __init__(self, linear_x=0.0, angular_z=0.0, stop=1):
        self.linear_x = linear_x # decimal (float)
        self.angular_z = angular_z # decimal (float)
        self.stop = stop # integer 0 or 1

class real_speed_struct(object):
    def __init__(self, speed_left=0.0, speed_right=0.0):
        self.speed_left = speed_left
        self.speed_right = speed_right

to_send = False
send_data = vel_struct()

if __name__ == '__main__':
    try:
        # initiate connection
        link = txfer.SerialTransfer('COM3')
        link.open()
        sleep(2)
        while True:
            # x, z, s = input('Command: ').split()
            # send_data = vel_struct(float(x), float(z), int(s))

            # receiving encoder speeds
            if(link.available()):
                packet_size = 0
                real_speed = real_speed_struct()
                real_speed.speed_left = link.rx_obj(obj_type='f', start_pos=packet_size)
                packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
                real_speed.speed_right = link.rx_obj(obj_type='f', start_pos=packet_size)
                packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
                print("Real speed received: {0}, {1}".format(real_speed.speed_left, real_speed.speed_right))
                # send response
                packet_size = 0
                packet_size = link.tx_obj("OK", packet_size)
                link.send(packet_size)

            # send commands
            if(to_send):
                # make packet
                packet_size = 0 
                packet_size = link.tx_obj(send_data.linear_x, start_pos=packet_size)
                packet_size = link.tx_obj(send_data.angular_z, start_pos=packet_size)
                packet_size = link.tx_obj(send_data.stop, start_pos=packet_size)
                # send packet
                link.send(packet_size)
                print("New packet sent: {0} , {1} , {2}".format(send_data.linear_x, send_data.angular_z, send_data.stop))
                # wait for string response
                while not link.available():
                    if link.status < 0:
                        if link.status == txfer.CRC_ERROR:
                            print('ERROR: CRC_ERROR')
                        elif link.status == txfer.PAYLOAD_ERROR:
                            print('ERROR: PAYLOAD_ERROR')
                        elif link.status == txfer.STOP_BYTE_ERROR:
                            print('ERROR: STOP_BYTE_ERROR')
                        else:
                            print('ERROR: {}'.format(link.status))
                packet_size = 0
                ok = link.rx_obj(obj_type=str, start_pos=packet_size, obj_byte_size=2)
                packet_size += len(ok)
                print("Response: {0}".format(ok))
                to_send = False
                
            # timeout
            # sleep(0.01)
    except KeyboardInterrupt:
        link.close()
    