#ifndef POINTS_FOR_LOOP_HPP
#define POINTS_FOR_LOOP_HPP
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

class PointsForLoop {
public:
    PointsForLoop(const std::string &node_name);

    void publishMarkers(); 

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::vector<geometry_msgs::msg::Point> sphere_positions_; // Konumları depolamak için bir vektör
    uint32_t shape_;
};

#endif  // POINTS_FOR_LOOP_HPP
