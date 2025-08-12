#include "moveit2_samples/position_server.hpp"
#include <functional>

MoveitServer::MoveitServer() : Node("server_node") {


    srv_ = this->create_service<moveit2_samples::srv::Position>("/position_topic",
    std::bind(&MoveitServer::server_call_back, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(),"Position server is ready.");

}

void MoveitServer::server_call_back(const std::shared_ptr<moveit2_samples::srv::Position::Request> request,
                         const std::shared_ptr<moveit2_samples::srv::Position::Response> response){

RCLCPP_INFO(this->get_logger(),"x: %.2f, y: %.2f, z: %.2f",request->x,request->y,request->z);
if(request->x < 100 && request->y < 100 && request->z < 100){
  response->success = true;
  response->message = "position accepted";
}

else{
response->success = false;
response->message = "limits is too large";
}

}


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}