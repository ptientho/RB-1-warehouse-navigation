#pragma once
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"
#include <functional>
#include <memory>
#include <string>

using namespace BT;
class GoToPoseDes : public RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
  using Pose = geometry_msgs::msg::PoseStamped;
  using Point = geometry_msgs::msg::PointStamped;
  GoToPoseDes(const std::string &name, const BT::NodeConfig &conf,
              const BT::RosNodeParams &params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {

    this->nh_ = params.nh;
    this->point_sub_ = nh_->create_subscription<Point>(
        "/clicked_point", 2,
        std::bind(&GoToPoseDes::point_callback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<double>("goal_degree")});
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

private:
  // clicked_point subscriber
  rclcpp::Subscription<Point>::SharedPtr point_sub_;
  rclcpp::Node::SharedPtr nh_;
  Point clicked_point;

  void point_callback(const std::shared_ptr<Point> msg) {

    if (msg == nullptr) {
      return;
    } else {
      clicked_point.header = msg->header;
      clicked_point.point = msg->point;
    }
  }
};