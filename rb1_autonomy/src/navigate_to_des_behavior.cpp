#include "rb1_autonomy/navigate_to_des_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "rb1_autonomy/action_node.h"
#include "rb1_autonomy/autonomy.h"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

using namespace std;

bool GoToPoseDes::setGoal(Goal &goal) {
  // Retrieve goal degree
  Expected<double> goal_degree = getInput<double>("goal_degree");

  // Convert degree to radian
  double goal_rad = goal_degree.value() * (M_PI / 180.0);

  // Create quaternion from radian
  tf2::Quaternion q;
  q.setRPY(0, 0, goal_rad);
  q.normalize();

  // Populate goal pose
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = node_->get_parameter("goal_x").as_double();
  goal.pose.pose.position.y = node_->get_parameter("goal_y").as_double();
  goal.pose.pose.orientation = tf2::toMsg(q);

  // publish goal frame
  publishGoalFrame(goal);

  // Log goal information
  RCLCPP_INFO(node_->get_logger(),
              "Goal Set to ---> x: %f, y: %f, theta: %f [rad]",
              goal.pose.pose.position.x, goal.pose.pose.position.y, goal_rad);

  return true;
}

BT::NodeStatus GoToPoseDes::onResultReceived(const WrappedResult &wr) {
  // Log result received
  RCLCPP_INFO(node_->get_logger(), "%s: Result Received.", name().c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
GoToPoseDes::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  // Log feedback
  RCLCPP_INFO(node_->get_logger(), "%s: Distance Remaining: %f", name().c_str(),
              feedback->distance_remaining);

  return BT::NodeStatus::RUNNING;
}

void GoToPoseDes::publishGoalFrame(const Goal &nav_pose) {

  auto t = geometry_msgs::msg::TransformStamped();
  t.header.frame_id = nav_pose.pose.header.frame_id;
  t.child_frame_id = "cart_goal";
  t.header.stamp = nav_pose.pose.header.stamp;

  t.transform.translation.x = nav_pose.pose.pose.position.x;
  t.transform.translation.y = nav_pose.pose.pose.position.y;
  t.transform.translation.z = nav_pose.pose.pose.position.z;
  t.transform.rotation.x = nav_pose.pose.pose.orientation.x;
  t.transform.rotation.y = nav_pose.pose.pose.orientation.y;
  t.transform.rotation.z = nav_pose.pose.pose.orientation.z;
  t.transform.rotation.w = nav_pose.pose.pose.orientation.w;

  RCLCPP_INFO(node_->get_logger(), "%s frame published",
              t.child_frame_id.c_str());
  tf_pub_->sendTransform(t);
}
