#ifndef _DIFF_BOT_HPP_
#define _DIFF_BOT_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "wheel.hpp"
#include "arduino_comms.hpp"

namespace midi_bot
{
    class MidiHardware : public hardware_interface::SystemInterface{

        struct Config{
            std::string left_wheel_name = "";
            std::string right_wheel_name = ""; 
            float loop_rate{0.0};
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            int enc_counts_per_rev = 0;
            double pid_p = 0.0;
            double pid_i = 0.0;
            double pid_d = 0.0;
            double pid_o = 0.0;
        };


    };
}







#endif //_DIFF_BOT_HPP_
