#ifndef _EMG_STOP_CLIENT_HPP_
#define _EMG_STOP_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/emergency_stop.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class EmgClientBase : public rclcpp::Node{

    public:

    EmgClientBase();

    ~EmgClientBase();

    bool sendEmg();

    private:

    rclcpp::Client<custom_interfaces::srv::EmergencyStop>::SharedPtr m_ServiceClient;

};



#endif // _EMG_STOP_CLIENT_HPP_