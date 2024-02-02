#include "rb1_autonomy/shelf_detection_real_behavior.h"
#include "behaviortree_cpp/basic_types.h"
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

using namespace std::chrono_literals;

bool ShelfDetectionRealClient::setRequest(Request::SharedPtr &request) {

  return true;
}

BT::NodeStatus ShelfDetectionRealClient::onResponseReceived(
    const Response::SharedPtr &response) {
  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | shelf_found: %s",
              name().c_str(), response->shelf_found ? "yes" : "no");
  RCLCPP_INFO(node_->get_logger(),
              "%s: Response received. | shelf_pose: X:%f Y:%f", name().c_str(),
              response->shelf_pose.pose.position.x,
              response->shelf_pose.pose.position.y);

  setOutput("find_shelf", response->shelf_found);
  setOutput("shelf_pose", response->shelf_pose);
  
  // check shelf_pose transform
  float x = response->shelf_pose.pose.position.x;
  float y = response->shelf_pose.pose.position.y;
  if (x == 0.0 && y == 0.0) {
    return BT::NodeStatus::FAILURE;
  }else {
    return BT::NodeStatus::SUCCESS;
  }

}
