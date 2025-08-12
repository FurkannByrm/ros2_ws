#include "moveit2_samples/position_client.hpp"

MoveitClient::MoveitClient() : Node{"client_node"},
                               PositionParser{"/home/cengo/jazzy_ws/src/moveit2_samples/config/position.json"} 
{ 
    
client_ = this->create_client<moveit2_samples::srv::Position>("/position_topic");


timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&MoveitClient::client_callback,this));

}


void MoveitClient::client_callback(){

if(position_.empty()){
    RCLCPP_INFO(this->get_logger(),"All positions processed.");
    timer_->cancel();
    return;
}

if(!client_->wait_for_service(std::chrono::seconds(1))){
    RCLCPP_WARN(this->get_logger(),"wait for the server to be up...");
    return;
}

auto pos = position_.front();
auto request = std::make_shared<moveit2_samples::srv::Position::Request>();
request->x = pos.x_;
request->y = pos.y_;
request->z = pos.z_;


auto future = client_->async_send_request(request,[this](rclcpp::Client<moveit2_samples::srv::Position>::SharedFuture future){

auto response = future.get();

if(response->success){
    RCLCPP_INFO(this->get_logger(),"Success : %s",response->message.c_str());
    position_.pop_front();
}

else{
    RCLCPP_INFO(this->get_logger(), "Faild : %s",response->message.c_str());
    position_.pop_front();
}

});

}


int main(int argc, char** args){
    rclcpp::init(argc,args);

    auto node = std::make_shared<MoveitClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}