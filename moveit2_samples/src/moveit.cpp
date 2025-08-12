#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // MoveGroupInterface oluştur (grup adını kendi robotunuza göre değiştirin)
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

  // Hedef pozisyonu belirle
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.3;
  move_group.setPoseTarget(target_pose);

  // Planla ve uygula
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group.execute(plan);
    RCLCPP_INFO(logger, "Plan başarıyla uygulandı!");
  } else {
    RCLCPP_ERROR(logger, "Planlama başarısız!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}