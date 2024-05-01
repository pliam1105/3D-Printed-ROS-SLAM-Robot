#include "slam_bot_hardware/arduino_hardware.hpp"

namespace slam_bot_hardware
{
    return_type ArduinoHardware::configure(const hardware_interface::HardwareInfo &info){

    }

    std::vector<hardware_interface::StateInterface> ArduinoHardware::export_state_interfaces(){

    }

    std::vector<hardware_interface::CommandInterface> ArduinoHardware::export_command_interfaces(){
        
    }

    return_type ArduinoHardware::start(){

    }

    return_type ArduinoHardware::stop(){

    }

    return_type ArduinoHardware::read(){

    }

    return_type ArduinoHardware::write(){

    }
    
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    slam_bot_hardware::ArduinoHardware,
    hardware_interface::SystemInterface
)