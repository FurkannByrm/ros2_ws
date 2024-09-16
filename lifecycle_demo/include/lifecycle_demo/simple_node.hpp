#ifndef SIMPLE_NODE_HPP_
#define SIMPLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class SimpleLifecycle : public rclcpp_lifecycle::LifecycleNode
{
    public:

    explicit SimpleLifecycle(const std::string & node_name, bool intra_process_comms);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state);


    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    void msgCallBack(const std_msgs::msg::String &msg);

};






#endif //SIMPLE_NODE_HPP_