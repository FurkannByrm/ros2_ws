#ifndef _STATIC_BROADCASTER_HPP_
#define __STATIC_BROADCASTER_HPP_

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node{

    public:
    explicit StaticFramePublisher(char* transformation[]);
    
    private:
    void make_transforms(char * transformation[]);
    std::__shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};


#endif //_STATIC_BROADCASTER_HPP_