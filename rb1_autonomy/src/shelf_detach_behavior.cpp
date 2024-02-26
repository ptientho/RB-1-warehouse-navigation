#include "rb1_autonomy/shelf_detach_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

bool DetachShelfClient::setRequest(Request::SharedPtr &request) {
  // Set request parameters
  getInput("detach_shelf", request->detach_shelf);
  RCLCPP_INFO(node_->get_logger(), "%s: Request sent. detach_shelf = %s",
              name().c_str(), request->detach_shelf ? "true" : "false");
  return true;
}

BT::NodeStatus DetachShelfClient::onResponseReceived(const Response::SharedPtr &response) {
  // Log response information
  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | detach shelf: %s",
              name().c_str(), response->is_success ? "yes" : "no");
  
  // Get YAML file name from input port
  auto attach_var = getInput<std::string>("shelf_attached");
  
  // Get file location
  const std::string yaml_file = node_->get_parameter("location_file").as_string();
  YAML::Node yaml = YAML::LoadFile(yaml_file);

  // Update is_attached variable
  yaml[attach_var.value()] = !response->is_success;

  // Write output to file
  std::ofstream fout(yaml_file);
  if (fout.is_open()) {
    fout << yaml;
    fout.close(); // Close the file
    RCLCPP_INFO(node_->get_logger(), "%s: Updated shelf attached status in %s",
                name().c_str(), yaml_file.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "%s: Error opening %s",
                 name().c_str(), yaml_file.c_str());
    return BT::NodeStatus::FAILURE;
  }
}
