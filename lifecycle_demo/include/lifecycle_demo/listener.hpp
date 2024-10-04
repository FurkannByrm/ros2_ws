#ifndef _LISTENER_HPP_
#define _LISTENER_HPP_
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class Listener : public rclcpp::Node
{
    public: 
    explicit Listener(const std::string & nodeName);
    
    private:
    void messageCallBack(const std_msgs::msg::String::SharedPtr msg)const;
    void notificationCallBack(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)const;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> messageSubscription;
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> notificationSubscription;

};


#endif //_LISTENER_HPP_