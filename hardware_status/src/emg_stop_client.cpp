#include "hardware_status/emg_stop_client.hpp"

EmgClientBase::EmgClientBase()
: Node("emg_stop_client")
{
    m_ServiceClient = this->create_client<custom_interfaces::srv::EmergencyStop>("emg_stop_server");
}

EmgClientBase::~EmgClientBase()
{
}

bool EmgClientBase::sendEmg()
{   
    if (!m_ServiceClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available");
        return false;
    }

    auto request = std::make_shared<custom_interfaces::srv::EmergencyStop::Request>();
    request->trigger = std_msgs::msg::Empty();

    auto future = m_ServiceClient->async_send_request(request);

    try {
        auto response = future.get();
        return response->stopped;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        return false;
    }
}
