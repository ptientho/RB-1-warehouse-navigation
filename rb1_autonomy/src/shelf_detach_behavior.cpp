#include "rb1_autonomy/shelf_detach_behavior.h"
#include "fstream"
#include "behaviortree_cpp/basic_types.h"
#include "rb1_autonomy/autonomy.h"
#include "rb1_autonomy/service_node.h"
#include "rclcpp/create_client.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <fstream>
#include <string>

using namespace std::chrono_literals;

/* New servide node */

bool DetachShelfClient::setRequest(Request::SharedPtr &request) {

  getInput("detach_shelf", request->detach_shelf);
  RCLCPP_INFO(node_->get_logger(), "%s: Request sent. detach_shelf = %s",
              name().c_str(), request->detach_shelf ? "true" : "false");
  return true;
}

BT::NodeStatus
DetachShelfClient::onResponseReceived(const Response::SharedPtr &response) {

  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | detach shelf: %s",
              name().c_str(), response->is_success ? "yes" : "no");
  // setOutput("is_success", response->is_success);
  //  get yaml file name from input port
  auto attach_var = getInput<std::string>("shelf_attached");
  // get file location
  const std::string yaml_file =
      node_->get_parameter("location_file").as_string();
  YAML::Node yaml = YAML::LoadFile(yaml_file);

  // update is_attached var
  if (response->is_success) {
    yaml[attach_var.value()] = false;

  } else {
    yaml[attach_var.value()] = true;
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
    RCLCPP_INFO(node_->get_logger(), "%s: Error opening %s",
              name().c_str(), yaml_file.c_str());
    return BT::NodeStatus::FAILURE;
  }

}