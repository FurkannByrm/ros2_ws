#include "lifecycle_demo/lifecycyle_talker.hpp"

LifecycleTalker::LifecycleTalker(const std::string & nodeName, bool intraProcessComms = false) 
: rclcpp_lifecycle::LifecycleNode(nodeName,rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms))
{

}

void LifecycleTalker::publish()
{
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "lifecycle hello world" + std::to_string(++count);
    if(!publisher->is_activated())
    {
    RCLCPP_INFO(this->get_logger(),"lifecycle publisher currently inactive");
    }
    else
    {
    RCLCPP_INFO(this->get_logger(),"lifecycle publisher active. Publishing [%s]", msg->data.c_str());
    }

    publisher->publish(std::move(msg));
}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleTalker::on_configure(const rclcpp_lifecycle::State &)
    {
    publisher = this->create_publisher<std_msgs::msg::String>("message",10);
    timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LifecycleTalker::publish, this));
    RCLCPP_INFO(this->get_logger(),"on configure() called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleTalker::on_activate(const rclcpp_lifecycle::State &)
    {
    publisher->on_activate();
    RCLCPP_INFO(this->get_logger(),"on_activate() called");   
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleTalker::on_deactivate(const rclcpp_lifecycle::State &)
    {
     
    publisher->on_deactivate();
    RCLCPP_INFO(this->get_logger(),"on_deactive() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleTalker::on_cleanup(const rclcpp_lifecycle::State &)
    {
    publisher.reset();
    timer.reset();
    RCLCPP_INFO(this->get_logger(), "on_cleanup() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   LifecycleTalker::on_shutdown(const rclcpp_lifecycle::State &)
    {
    
    RCLCPP_INFO(this->get_logger(),"on_shutdown() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

 


int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<LifecycleTalker> lc_publisher_node = std::make_shared<LifecycleTalker>("lc_talker");

    executor.add_node(lc_publisher_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}