#ifndef _ARDUINO_COMMS_HPP_
#define _ARDUINO_COMMS_HPP_

#include <libserial/SerialPort.h>
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <iostream>

class ControlBoardComms : public rclcpp::Node
{
    public: 
    
    ControlBoardComms() = default;

    void connect(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
    void disconnect();
    bool connected() const;
    LibSerial::BaudRate convert_baud_rate(int baud_rate);

    std::string send_msg(const std::string & msg_to_send, bool print_output);
    void send_empty_msg();

    void read_encoder_values(int &val_1, int &val_2);
    void set_motor_values(int val_1, int val_2);
    void set_pid_values(double k_p, double k_d, double k_i, double k_o);

    private:
    LibSerial::SerialPort serial_conn_;
    double timeout_ms_;
};


#endif //_ARDUINO_COMMS_HPP_