#include "rb1_autonomy/shelf_attach_behavior.h"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rb1_autonomy/autonomy.h"
#include "rb1_autonomy/service_node.h"
#include "rclcpp/create_client.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <string>
#include "fstream"

using namespace std::chrono_literals;

/* New servide node */

bool AttachShelfClient::setRequest(Request::SharedPtr &request) {

  getInput("attach_shelf", request->attach_shelf);
  getInput("front_distance", request->front_distance);
  RCLCPP_INFO(node_->get_logger(), "%s: Request sent. attach_shelf = %s",
              name().c_str(), request->attach_shelf ? "true" : "false");
  return true;
}

BT::NodeStatus
AttachShelfClient::onResponseReceived(const Response::SharedPtr &response) {

  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | attach shelf: %s",
              name().c_str(), response->is_success ? "yes" : "no");
  
  auto attach_var = getInput<std::string>("shelf_attached");
  // get file location
  const std::string yaml_file =
      node_->get_parameter("location_file").as_string();
  YAML::Node yaml = YAML::LoadFile(yaml_file);

  // update is_attached var
  if (response->is_success) {
    yaml[attach_var.value()] = true;

  } else {
    yaml[attach_var.value()] = false;
  }

  // write output to file
  std::ofstream fout(yaml_file);
  if (fout.is_open()) {
    // Write the XML string to the file
    fout << yaml;

    // Close the file
    fout.close();

    RCLCPP_INFO(node_->get_logger(), "%s: Update shelf attached status in %s",
                name().c_str(), yaml_file.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(node_->get_logger(), "%s: Error opening %s", name().c_str(),
                yaml_file.c_str());
    return BT::NodeStatus::FAILURE;
  }

}