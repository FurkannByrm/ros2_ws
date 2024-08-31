#ifndef _BATTERY_STATUS_PUBLISHER_HPP_
#define _BATTERY_STATUS_PUBLISHER_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/bms_status.hpp"

class BatteryStatus : public rclcpp::Node{

    public:
    BatteryStatus();

    private:

    void BatteryStatusCallBack();
    rclcpp::Publisher<custom_interfaces::msg::BmsStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;



};

#endif //_BATTERY_STATUS_PUBLISHER_HPP_
