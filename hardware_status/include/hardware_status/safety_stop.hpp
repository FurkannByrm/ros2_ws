#ifndef _SAFETY_STOP_HPP_
#define _SAFETY_STOP_HPP_


#include <chrono>
#include <thread> 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"

using std::placeholders::_1;



enum class State{

    FREE = 0,
    WARNING,
    DANGER

};

class SafetyStop : public rclcpp::Node{

    public:
    SafetyStop();    

    private:

    double warning_distance_, danger_distance_;
    bool is_first_msg_;
    State state_, prev_state_;
    visualization_msgs::msg::MarkerArray zones_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

    void laserCallBack(const sensor_msgs::msg::LaserScan &msg);


};



#endif //_SAFETY_STOP_HPP_