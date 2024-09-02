#include "hardware_status/safety_stop.hpp"


SafetyStop::SafetyStop(): Node{"safety_stop_node"}, is_first_msg_{true}, state_{State::FREE}, prev_state_{State::FREE}
{
    RCLCPP_INFO(this->get_logger(),"Node initializated");
    declare_parameter<double>("warning_distance",0.6);
    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<std::string>("scan_topic","scan");
    declare_parameter<std::string>("safety_stop_topic","safety_stop");
    warning_distance_ = get_parameter("warning_distance").as_double();
    danger_distance_ = get_parameter("danger_distance")
    
}



int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyStop>());
    rclcpp::shutdown();
    return 0;

}

