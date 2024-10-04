#include "lifecycle_demo/listener.hpp"

Listener::Listener(const std::string & nodeName) : Node(nodeName) {

messageSubscription = this->create_subscription<std_msgs::msg::String>("messages", 10, std::bind(&Listener::messageCallBack,this, std::placeholders::_1));
notificationSubscription = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("lc_talker/transition_event", 10, std::bind(&Listener::notificationCallBack, this, std::placeholders::_1));
}

void Listener::messageCallBack(const std_msgs::msg::String::SharedPtr msg)const
{
    RCLCPP_INFO(this->get_logger(),"messageCallBack : %s", msg->data.c_str());
}

void Listener::notificationCallBack(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)const
{
    RCLCPP_INFO(this->get_logger(), "notificationCallBack: %s to %s",msg->start_state.label.c_str(),msg->goal_state.label.c_str());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto listener_node = std::make_shared<Listener>("listener_node");
    rclcpp::spin(listener_node);
    rclcpp::shutdown();

    return 0;


}