#ifndef _TF_KINEMATIC_HPP
#define _TF_KINEMATIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "custom_interfaces/srv/get_transform.hpp"
#include  <memory>

class TfKinematics : public rclcpp::Node
{
    public: 
    TfKinematics(const std::string& name);

    private:
    void timerCallBack();

    bool getTransformCallBack(const std::shared_ptr<custom_interfaces::srv::GetTransform::Request> req,
                              const std::shared_ptr<custom_interfaces::srv::GetTransform::Response> res);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<custom_interfaces::srv::GetTransform>::SharedPtr get_transform_srv_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;
    double x_increment_;
    double last_x_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
    int rotations_counter_;



};

#endif //TF_KINEMATICS_HPP