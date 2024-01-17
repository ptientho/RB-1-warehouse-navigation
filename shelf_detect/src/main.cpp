#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "shelf_detect/shelf_detect_server.hpp"
#include "shelf_detect/shelf_detect_server_real.hpp"
#include <memory>
#include <string>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  // how to define argument here
  ///////////////////////////////////////////////////////////
  std::string message = argv[2];
  bool is_real_robot = message != "false";
  ///////////////////////////////////////////////////////////
  rclcpp::Node::SharedPtr node;
  if (is_real_robot) {
    node = std::make_shared<ShelfDetectionServerReal>();
    RCLCPP_INFO(node->get_logger(), "Running Shelf Detection Real...");
  } else {
    node = std::make_shared<ShelfDetectionServer>();
    RCLCPP_INFO(node->get_logger(), "Running Shelf Detection Sim...");
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}