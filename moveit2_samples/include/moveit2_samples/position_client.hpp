#include "rclcpp/rclcpp.hpp"
#include "moveit2_samples/srv/position.hpp"
#include "moveit2_samples/position_parser.hpp"


class MoveitClient : public rclcpp::Node, PositionParser{
   public:
    MoveitClient();
    private:
    void client_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<moveit2_samples::srv::Position>::SharedPtr client_;
       
};