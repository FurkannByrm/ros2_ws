#include "rclcpp/rclcpp.hpp"
#include "moveit2_samples/srv/position.hpp"



class MoveitServer : public rclcpp::Node {
   public:
    MoveitServer();
    private:
    void server_call_back(std::shared_ptr<moveit2_samples::srv::Position::Request> request,
                         std::shared_ptr<moveit2_samples::srv::Position::Response> response);
    rclcpp::Service<moveit2_samples::srv::Position>::SharedPtr srv_;
  
};