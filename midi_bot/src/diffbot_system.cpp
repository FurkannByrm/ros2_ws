#include "midi_bot/diffbot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace midi_bot
{

hardware_interface::CallbackReturn MidiHardware::on_init(const hardware_interface::HardwareInfo& info){
if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
{   
    return hardware_interface::CallbackReturn::ERROR;
}

cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
cfg_.device = info_.hardware_parameters["device"];
cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeot_ms"]);
cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
if(info.hardware_parameters.count("pid_p") > 0)
{
    cfg_.pid_p = std::stod(info_.hardware_parameters["pid_p"]);
    cfg_.pid_i = std::stod(info_.hardware_parameters["pid_i"]);
    cfg_.pid_d = std::stod(info_.hardware_parameters["pid_d"]);
    cfg_.pid_o = std::stod(info_.hardware_parameters["pid_o"]);
}
else
{
    RCLCPP_INFO(this->get_logger(), "PID values not supplied, using defaults.");
}
 
wheel_l_.setup(cfg_.left_wheel_name,cfg_.enc_counts_per_rev);
Wheel_r_.setup(cfg_.right_wheel_name,cfg_.enc_counts_per_rev);

for(const hardware_interface::ComponentInfo& joint : info_.joints)
{
    if(joint.command_interfaces.size() != 1 )
    {
    RCLCPP_FATAL(this->get_logger(),"joint '%s' has %zu command interfaces found. 1 expected.", 
    joint.name.c_str(), joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
    }

    if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
    RCLCPP_FATAL(this->get_logger(),"Joint '%s' has %s command interfaces found.",
    joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
    return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
    RCLCPP_FATAL(this->get_logger(), "joint '%s' has %zu state interface. 2 expected.",
     joint.name.c_str(), joint.state_interfaces.size());
     return hardware_interface::CallbackReturn::ERROR;
    }

    if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
    RCLCPP_FATAL(this->get_logger(),"joint '%s' have '%s' as first state interface. '%s' expected.",
    joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
    }
    
    if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
        RCLCPP_FATAL(this->get_logger(),"Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY );
    }
    
}


}

















}