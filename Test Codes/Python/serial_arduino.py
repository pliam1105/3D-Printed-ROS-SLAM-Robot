from time import sleep
from pySerialTransfer import pySerialTransfer as txfer

class vel_struct(object):
    def __init__(self, linear_x=0.0, angular_z=0.0, stop=1):
        self.linear_x = linear_x # decimal (float)
        self.angular_z = angular_z # decimal (float)
        self.stop = stop # integer 0 or 1

if __name__ == '__main__':
    try:
        # initiate connection
        link = txfer.SerialTransfer('COM3')
        link.open()
        sleep(2)
        while True:
            x, z, s = input('Command: ').split()
            send_data = vel_struct(float(x), float(z), int(s))
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
            receive_data = vel_struct()
            receive_data.linear_x = link.rx_obj(obj_type='f', start_pos=packet_size)
            packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
            receive_data.angular_z = link.rx_obj(obj_type='f', start_pos=packet_size)
            packet_size += txfer.STRUCT_FORMAT_LENGTHS['f']
            receive_data.stop = link.rx_obj(obj_type='i', start_pos=packet_size)
            packet_size += txfer.STRUCT_FORMAT_LENGTHS['i']
            ok = link.rx_obj(obj_type=str, start_pos=packet_size, obj_byte_size=2)
            packet_size += len(ok)
            print("Response: {0} , {1} , {2}, {3}".format(receive_data.linear_x, receive_data.angular_z, receive_data.stop, ok))
    except KeyboardInterrupt:
        link.close()
    