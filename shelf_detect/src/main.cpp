#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "shelf_detect/shelf_detect_server.hpp"
#include <memory>
#include <string>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;
  node = std::make_shared<ShelfDetectionServer>();
  RCLCPP_INFO(node->get_logger(), "Running Shelf Detection Sim...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}