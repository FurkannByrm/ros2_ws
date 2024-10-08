#include "rviz_plugins/points_for_loop.hpp"

PointsForLoop::PointsForLoop(const std::string &node_name) {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared(node_name);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
    shape_ = visualization_msgs::msg::Marker::SPHERE;

    // Pozisyonları ekle
    geometry_msgs::msg::Point point1;
    point1.x = 2.0; point1.y = 2.0; point1.z = 0.0;
    sphere_positions_.push_back(point1);

    geometry_msgs::msg::Point point2;
    point2.x = -2.0; point2.y = 2.0; point2.z = 0.0;
    sphere_positions_.push_back(point2);

    geometry_msgs::msg::Point point3;
    point3.x = -2.0; point3.y = -2.0; point3.z = 0.0;
    sphere_positions_.push_back(point3);

    geometry_msgs::msg::Point point4;
    point4.x = 2.0; point4.y = -2.0; point4.z = 0.0;
    sphere_positions_.push_back(point4);
}

void PointsForLoop::publishMarkers() {
    rclcpp::Rate loop_rate(1); // 1 Hz

    while (rclcpp::ok()) {
        for (size_t i = 0; i < sphere_positions_.size(); ++i) {
            visualization_msgs::msg::Marker marker;

            // Marker'ın frame ID ve zaman damgasını ayarla
            marker.header.frame_id = "map";  // "/map" yerine "map" kullanın
            marker.header.stamp = rclcpp::Clock().now();

            // Marker ID ve namespace ayarla
            marker.ns = "basic_shapes";
            marker.id = i;

            // Marker tipi ve aksiyonu ayarla
            marker.type = shape_;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Marker'ın konumunu ayarla
            marker.pose.position = sphere_positions_[i];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Marker'ın boyutunu ayarla
            marker.scale.x = 0.5; // Boyutları biraz artırdık
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            // Marker'ın rengini yeşil olarak ayarla
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            // Marker'ın süresini sonsuz yap
            marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

            // Marker'ı yayınla
            marker_pub_->publish(marker);
        }

        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    PointsForLoop points_publisher("points_for_loop");
    points_publisher.publishMarkers();

    return 0;
}
