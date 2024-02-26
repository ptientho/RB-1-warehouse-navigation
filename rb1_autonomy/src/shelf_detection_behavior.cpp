#include "rb1_autonomy/shelf_detection_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include <cmath>
#include <string>

using namespace std::chrono_literals;

bool ShelfDetectionClient::setRequest(Request::SharedPtr &request) {
  // Set request parameters
  getInput("front_offset", request->front_offset);
  return true;
}

BT::NodeStatus
ShelfDetectionClient::onResponseReceived(const Response::SharedPtr &response) {
  // Log response information
  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | shelf_found: %s",
              name().c_str(), response->shelf_found ? "yes" : "no");
  RCLCPP_INFO(node_->get_logger(),
              "%s: Response received. | shelf_pose: X:%f Y:%f", name().c_str(),
              response->shelf_pose.pose.position.x,
              response->shelf_pose.pose.position.y);
  // Set output parameters based on response
  setOutput("find_shelf", response->shelf_found);
  setOutput("shelf_pose", response->shelf_pose);

  // Return node status based on response
  return response->shelf_found ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
}
