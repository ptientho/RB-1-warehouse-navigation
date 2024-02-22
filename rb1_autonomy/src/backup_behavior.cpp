#include "rb1_autonomy/backup_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cmath>
#include <ratio>
#include <string>
#include <unistd.h>

using namespace std::chrono_literals;
Pose BackUpActionNode::get_tf(std::string fromFrame, std::string toFrame) {

  geometry_msgs::msg::TransformStamped t;
  auto shelf_pose = geometry_msgs::msg::PoseStamped();
  try {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {

    RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
                fromFrame.c_str(), toFrame.c_str(), ex.what());
    rclcpp::Clock::SharedPtr clock = node_->get_clock();
    shelf_pose.header.stamp = clock->now();
    shelf_pose.header.frame_id = "robot_base_link";
    shelf_pose.pose.position.x = 0.0;
    shelf_pose.pose.position.y = 0.0;
    shelf_pose.pose.position.z = 0.0;
    shelf_pose.pose.orientation.x = 0.0;
    shelf_pose.pose.orientation.y = 0.0;
    shelf_pose.pose.orientation.z = 0.0;
    shelf_pose.pose.orientation.w = 1.0;
    tf_success = false;
    return shelf_pose;
  }

  auto translation_pose = t.transform.translation;
  auto rotation_pose = t.transform.rotation;

  shelf_pose.pose.position.x = translation_pose.x;
  shelf_pose.pose.position.y = translation_pose.y;
  shelf_pose.pose.position.z = translation_pose.z;
  shelf_pose.pose.orientation.x = rotation_pose.x;
  shelf_pose.pose.orientation.y = rotation_pose.y;
  shelf_pose.pose.orientation.z = rotation_pose.z;
  shelf_pose.pose.orientation.w = rotation_pose.w;
  tf_success = true;
  return shelf_pose;
}

NodeStatus BackUpActionNode::tick() {

  auto backUpDist = getInput<double>("backup_dist").value();
  auto fromFrame = getInput<std::string>("from_frame").value();
  auto toFrame = getInput<std::string>("to_frame").value();

  const float VEL = 0.05;
  CmdVel vel_msg;
  rclcpp::Rate loop_rate(10);

  Pose pose = get_tf(fromFrame, toFrame);
  const float DISTANCE = (float)backUpDist;
  const float TOLERANCE = 0.05;
  float current_x = pose.pose.position.x;
  float current_y = pose.pose.position.y;

  // loop to ensure that get_tf receive transform
  while (!tf_success) {
    pose = get_tf(fromFrame, toFrame);
    current_x = pose.pose.position.x;
    current_y = pose.pose.position.y;
    loop_rate.sleep();
  }
  // safe guard before running loop
  if ((abs(current_x) > TOLERANCE) || (abs(current_y) > TOLERANCE)) {
    return NodeStatus::FAILURE;
  }
  while (current_x < DISTANCE) {
    // update
    pose = get_tf(fromFrame, toFrame);
    current_x = pose.pose.position.x;
    current_y = pose.pose.position.y;
    RCLCPP_INFO(node_->get_logger(), "X: %f, Y: %f", current_x, current_y);
    vel_msg.linear.x = (-1) * VEL;
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }

  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);

  return NodeStatus::SUCCESS;
}
