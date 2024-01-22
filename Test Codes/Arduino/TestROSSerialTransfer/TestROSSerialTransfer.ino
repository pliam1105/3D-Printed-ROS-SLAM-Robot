#include "SerialTransfer.h"

SerialTransfer ros_transfer;

struct __attribute__((packed)) vel_struct{
  float linear_x, angular_z;
  int32_t stop;
};

void setup() {
  Serial.begin(115200);
  ros_transfer.begin(Serial);
}

void loop() {
  if(ros_transfer.available()){
    //receive packet
    uint16_t packet_size = 0;
    vel_struct new_vel = vel_struct();
    packet_size = ros_transfer.rxObj(new_vel, packet_size);
    //send response
    char ok[] = "OK";
    packet_size = 0;
    packet_size = ros_transfer.txObj(new_vel.linear_x, packet_size);
    packet_size = ros_transfer.txObj(new_vel.angular_z, packet_size);
    packet_size = ros_transfer.txObj(new_vel.stop, packet_size);
    packet_size = ros_transfer.txObj(ok, packet_size);
    ros_transfer.sendData(packet_size);
  }
}
