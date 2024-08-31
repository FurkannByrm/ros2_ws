#include "hardware_status/battery_status_publisher.hpp"


BatteryStatus::BatteryStatus(): Node{"Bms_status_publisher_node"}
{
    publisher_ = this->create_publisher<custom_interfaces::msg::BmsStatus>("bms_status",10);
    timer_     = this->create_wall_timer(std::chrono::seconds(1),std::bind(&BatteryStatus::BatteryStatusCallBack,this));
}


void BatteryStatus::BatteryStatusCallBack()
{
    custom_interfaces::msg::BmsStatus msg;
    msg.battery_voltage = 1.2;
    publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(),"Robot voltage is :  %f",msg.battery_voltage);
}


int main(int argc, char ** args)
{
    rclcpp::init(argc, args);
    rclcpp::spin(std::make_shared<BatteryStatus>());
    rclcpp::shutdown();
    
}