#include "rb1_autonomy/navigation_pose_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "rb1_autonomy/action_node.h"
#include "rb1_autonomy/autonomy.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include <functional>
#include <string>
#include <vector>


bool GoToPose2ActionClient::setGoal(Goal &goal) {

  // get the key corresponding to "loc"
  Expected<Pose> shelf_pose = getInput<Pose>("pose");

  // shelf_pose is type geometry_msgs::msg::PoseStamped

  // build new goal based on goal_ variable
  auto goal_ = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = shelf_pose->pose.position.x;
  goal.pose.pose.position.y = shelf_pose->pose.position.y;
  goal.pose.pose.orientation.x = shelf_pose->pose.orientation.x;
  goal.pose.pose.orientation.y = shelf_pose->pose.orientation.y;
  goal.pose.pose.orientation.z = shelf_pose->pose.orientation.z;
  goal.pose.pose.orientation.w = shelf_pose->pose.orientation.w;
  
  return true;
}

BT::NodeStatus
GoToPose2ActionClient::onResultReceived(const WrappedResult &wr) {

  RCLCPP_INFO(node_->get_logger(), "%s: Result Received.", name().c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoToPose2ActionClient::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {

  RCLCPP_INFO(node_->get_logger(), "%s: Distance Remaining: %f", name().c_str(),
              feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}