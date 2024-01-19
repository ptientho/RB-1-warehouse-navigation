#pragma once
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>
#include <string>

using namespace BT;
class GoToPose2ActionClient
    : public RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
  using Pose = geometry_msgs::msg::PoseStamped;
  GoToPose2ActionClient(const std::string &name, const BT::NodeConfig &conf,
                        const BT::RosNodeParams &params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<Pose>("pose")});
  }

  bool setGoal(Goal &goal) override;

  void onHalt() override{};

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) {

    RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s",
                 name().c_str(), toStr(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus
  onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};