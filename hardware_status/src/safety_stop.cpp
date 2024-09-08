#include "hardware_status/safety_stop.hpp"


SafetyStop::SafetyStop(): Node{"safety_stop_node"}, is_first_msg_{true}, state_{State::FREE}, prev_state_{State::FREE}
{
    RCLCPP_INFO(this->get_logger(),"Node initializated");

    declare_parameter<double>("warning_distance", 2.0);
    declare_parameter<double>("danger_distance", 1.0);
    declare_parameter<std::string>("scan_topic","scan");
    declare_parameter<std::string>("safety_stop_topic","safety_stop");
    warning_distance_ = get_parameter("warning_distance").as_double();
    danger_distance_ = get_parameter("danger_distance").as_double();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    std::string safety_stop_topic = get_parameter("safety_stop_topic").as_string();

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&SafetyStop::laserCallBack, this, _1));
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic, 10);
    zones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("zones",10);
    decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this,"/a200_0000/joy_turbo_decrease");
    increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this,"/a200_0000/joy_turbo_increase");

    while(!decrease_speed_client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok())
    {
    RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available! Waiting..");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    while(!increase_speed_client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok())
    {
    RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available! Waiting..");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    }


    visualization_msgs::msg::Marker warning_zone;
    warning_zone.id = 0;
    warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
    warning_zone.action = visualization_msgs::msg::Marker::ADD;
    warning_zone.scale.z = 0.0001;
    warning_zone.scale.y = warning_distance_*2;
    warning_zone.scale.x = warning_distance_*2;
    warning_zone.color.r = 1.00;
    warning_zone.color.g = 0.984;
    warning_zone.color.b = 0.0;
    warning_zone.color.a = 0.5;
    zones_.markers.push_back(warning_zone);

    visualization_msgs::msg::Marker danger_zone = warning_zone;
    danger_zone.id = 1;
    danger_zone.scale.x = danger_distance_ * 2;
    danger_zone.scale.y = danger_distance_ * 2;
    danger_zone.color.r = 1.0;
    danger_zone.color.g = 0.0;
    danger_zone.color.b = 0.0;
    danger_zone.color.a = 0.5;
    danger_zone.pose.position.z = 0.001;
    zones_.markers.push_back(danger_zone);
}

void SafetyStop::laserCallBack(const sensor_msgs::msg::LaserScan &msg)
{

    
    state_ = State::FREE;
    for(const auto &range : msg.ranges)
    {
        if(range<=warning_distance_)
        {
            state_ = State::WARNING;
            RCLCPP_INFO(this->get_logger(), "WARNING DISTANCE!!!!!!");
            
            if(range<=danger_distance_)
            {

                state_ = State::DANGER;
                RCLCPP_INFO(this->get_logger(),"DANGER DISTANCE!!!!!");
                break;
            }

        }

    }

    if(state_ != prev_state_)
    {
        std_msgs::msg::Bool is_safety_stop;
        if(state_ == State::WARNING)
        {
            RCLCPP_INFO(this->get_logger(), "WARNING DISTANCE!!!!!!");
            is_safety_stop.data = true;
            decrease_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
            zones_.markers.at(0).color.a = 1;
            zones_.markers.at(1).color.a = 0.5;
        }
        else if(state_==State::DANGER)
        {
            RCLCPP_INFO(this->get_logger(),"DANGER DISTANCE!!!!!");
            is_safety_stop.data = true;
            zones_.markers.at(0).color.a = 1.0;
            zones_.markers.at(1).color.a = 1.0; 
        }
        else if(state_==State::FREE)
        {   
            RCLCPP_INFO(this->get_logger(),"FREE DISTANCE!!!!!");
            is_safety_stop.data = false;
            increase_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
            zones_.markers.at(0).color.a = 0.5;
            zones_.markers.at(1).color.a = 0.5;
        }

        prev_state_ = state_;
        safety_stop_pub_->publish(is_safety_stop);
    }

    if(is_first_msg_)
    {
        for(auto & zone : zones_.markers)
        {
            zone.header.frame_id;
        }
        is_first_msg_ = false;
    }
        zones_pub_->publish(zones_);

}


int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyStop>());
    rclcpp::shutdown();
    return 0;

}

