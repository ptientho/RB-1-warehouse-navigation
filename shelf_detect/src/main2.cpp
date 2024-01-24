#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "shelf_detect/shelf_detect_server_real.hpp"
#include <memory>
#include <string>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;
  node = std::make_shared<ShelfDetectionServerReal>();
  RCLCPP_INFO(node->get_logger(), "Running Shelf Detection Real...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}