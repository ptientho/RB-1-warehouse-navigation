#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rb1_autonomy/autonomy.h"
#include "rb1_autonomy/shelf_detection_behavior.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>
#include "behaviortree_ros2/bt_action_node.hpp"

using namespace BT;
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomyEngine>("autonomy_node");
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->registerNodes();
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}