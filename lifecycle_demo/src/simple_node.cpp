#include "lifecycle_demo/simple_node.hpp"



SimpleLifecycle::SimpleLifecycle(const std::string & node_name, bool intra_process_comms = false)
: rclcpp_lifecycle::LifecycleNode(node_name,
rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifecycle::on_configure(const rclcpp_lifecycle::State &state)
{
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&SimpleLifecycle::msgCallBack, this, _1));
    RCLCPP_INFO(this->get_logger(), "lifecycle node on_configure() called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifecycle::on_activate(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_configure(state);
    RCLCPP_INFO(this->get_logger(), "lifecycle node on_activate() called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifecycle::on_deactivate(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(this->get_logger(),"Lifeycyle node on_deactivate() called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifecycle::on_cleanup(const rclcpp_lifecycle::State &state)
{
    sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Lifecycle node on_cleanup() called." );
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SimpleLifecycle::on_shutdown(const rclcpp_lifecycle::State &state)
{
    sub_.reset();
    LifecycleNode::on_shutdown(state);
    RCLCPP_INFO(this->get_logger(), "Lifecycle_node on_shutdown() called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SimpleLifecycle::msgCallBack(const std_msgs::msg::String &msg)
{
    auto state = get_current_state();
    if(state.label() == "activate")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "lifecycle node heard: "<<msg.data.c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleLifecycle> simple_node = std::make_shared<SimpleLifecycle>("simple_node");
    ste.add_node(simple_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0; 
}
