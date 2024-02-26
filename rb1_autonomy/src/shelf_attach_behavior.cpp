#include "rb1_autonomy/shelf_attach_behavior.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

bool AttachShelfClient::setRequest(Request::SharedPtr &request) {
  // Set request parameters
  getInput("attach_shelf", request->attach_shelf);
  getInput("front_distance", request->front_distance);
  RCLCPP_INFO(node_->get_logger(), "%s: Request sent. attach_shelf = %s",
              name().c_str(), request->attach_shelf ? "true" : "false");
  return true;
}

BT::NodeStatus AttachShelfClient::onResponseReceived(const Response::SharedPtr &response) {
  // Log response information
  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | attach shelf: %s",
              name().c_str(), response->is_success ? "yes" : "no");
  
  // Get shelf attached variable name
  auto attach_var = getInput<std::string>("shelf_attached");
  
  // Get file location
  const std::string yaml_file = node_->get_parameter("location_file").as_string();
  YAML::Node yaml = YAML::LoadFile(yaml_file);

  // Update is_attached variable
  yaml[attach_var.value()] = response->is_success;

  // Write output to file
  std::ofstream fout(yaml_file);
  if (fout.is_open()) {
    fout << yaml; // Write YAML data to the file
    fout.close(); // Close the file
    RCLCPP_INFO(node_->get_logger(), "%s: Updated shelf attached status in %s",
                name().c_str(), yaml_file.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "%s: Error opening %s", name().c_str(),
                 yaml_file.c_str());
    return BT::NodeStatus::FAILURE;
  }
}
