#ifndef SLAM_BOT_HARDWARE__ARDUINO_HARDWARE_HPP_
#define SLAM_BOT_HARDWARE__ARDUINO_HARDWARE_HPP_

#include "slam_bot_hardware/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using hardware_interface::return_type;

namespace slam_bot_hardware
{
  class ArduinoHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>{
    public:
      return_type configure(const hardware_interface::HardwareInfo &info) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type start() override;

      return_type stop() override;

      return_type read() override;

      return_type write() override;
      
    private:

  };
}

#endif  // SLAM_BOT_HARDWARE__ARDUINO_HARDWARE_HPP_
