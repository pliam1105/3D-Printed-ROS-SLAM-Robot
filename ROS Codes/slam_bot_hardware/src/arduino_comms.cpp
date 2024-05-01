#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <slam_bot_hardware/arduino_comms.hpp>

namespace slam_bot_hardware{
    void ArduinoComms::initialize(){

    }

    void ArduinoComms::handshake(){

    }

    void ArduinoComms::update_motors(int left_speed, int right_speed){

    }

    void ArduinoComms::update_servos(int servo_angles[4]){

    }

    void ArduinoComms::read_motor_speeds(int &left_speed, int &right_speed){

    }

    void ArduinoComms::ensure_get_send(){

    }

    void ArduinoComms::stop(){

    }
}