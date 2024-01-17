#include "rb1_autonomy/shelf_detection_real_behavior.h"
//#include "behaviortree_cpp_v3/action_node.h"
//#include "behaviortree_cpp_v3/actions/set_blackboard_node.h"
//#include "behaviortree_cpp_v3/basic_types.h"
//#include "behaviortree_cpp_v3/blackboard.h"
//#include "behaviortree_cpp_v3/tree_node.h"
//#include "behaviortree_cpp/basic_types.h"
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

BT::NodeStatus
ShelfDetectionRealClient::onResponseReceived(const Response::SharedPtr &response) {
  RCLCPP_INFO(node_->get_logger(), "%s: Response received. | shelf_found: %s",
              name().c_str(), response->shelf_found ? "yes" : "no");
  setOutput("find_shelf", response->shelf_found);
  setOutput("shelf_pose", response->shelf_pose);
  return BT::NodeStatus::SUCCESS;
}

