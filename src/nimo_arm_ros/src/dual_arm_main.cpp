#include "nimo_arm_ros/dual_arm_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nimo_arm_ros::DualArmNode>());
  rclcpp::shutdown();
  return 0;
}
