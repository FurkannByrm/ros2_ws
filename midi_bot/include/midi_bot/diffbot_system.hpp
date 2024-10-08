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
#include "control_board_comms.hpp"
#include "visibility_control.hpp"

#include <limits>
#include <memory>
#include <vector>

namespace midi_bot
{
    class MidiHardware : public hardware_interface::SystemInterface, rclcpp::Node{

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

        public:

        RCLCPP_SHARED_PTR_DEFINITIONS(MidiHardware);

        MIDI_BOT_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        
        MIDI_BOT_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        MIDI_BOT_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        MIDI_BOT_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        MIDI_BOT_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        MIDI_BOT_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        MIDI_BOT_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;


        MIDI_BOT_PUBLIC 
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration& period) override;

        MIDI_BOT_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration& period) override;
        
        private:

        ControlBoardComms comms_;
        Config cfg_;
        Wheel wheel_l_;
        Wheel Wheel_r_;
    };
} //namespace midi_bot

#endif //_DIFF_BOT_HPP_
