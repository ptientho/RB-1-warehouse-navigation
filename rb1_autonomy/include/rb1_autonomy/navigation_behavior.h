#pragma once
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>
#include <string>
#include "rclcpp_action/rclcpp_action.hpp"

/*
class GoToPose : public BT::StatefulActionNode
{

public:
  GoToPose(const std::string &name, const BT::NodeConfiguration &config,
rclcpp::Node::SharedPtr node);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  bool done_flag_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  // action client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);

  ~GoToPose(){};

};
*/

/* Old version of GoToPoseActionClient
class GoToPoseActionClient
    : public ActionNode<nav2_msgs::action::NavigateToPose> {

public:
  GoToPoseActionClient(const std::string &xml_tag_name,
                       const std::string &action_name,
                       const BT::NodeConfiguration &conf, const
rclcpp::Node::SharedPtr node);

  rclcpp::Node::SharedPtr node_;
  void on_tick() override;

  void on_wait_for_result(
      std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
          feedback) override;

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<std::string>("loc")});
  }
  ~GoToPoseActionClient() {}
};
*/

/* New version of GoToPoseActionClient using behaviortree_ros2 pkg*/
using namespace BT;
class GoToPoseActionClient
    : public RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
  GoToPoseActionClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<std::string>("loc")});
  }

  bool setGoal(Goal& goal) override;

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