#pragma once
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <ratio>
#include <string>

using namespace BT;
using CmdVel = geometry_msgs::msg::Twist;
using Pose = geometry_msgs::msg::PoseStamped;
class BackUpActionNode : public SyncActionNode {

public:
  BackUpActionNode(const std::string &name, const NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node)
      : SyncActionNode(name, config), node_(node) {

    this->vel_pub_ = node_->create_publisher<CmdVel>("/cmd_vel", 10);
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    this->tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  virtual NodeStatus tick() override;
  // virtual void halt() override final;

  static BT::PortsList providedPorts() {
    // Define the ports provided by your node
    return {{BT::InputPort<double>("backup_dist"),
             BT::InputPort<std::string>("from_frame"),
             BT::InputPort<std::string>("to_frame")}};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  bool tf_success = false;

  // lstener to tf from robot_base_link to temp_cart
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Pose get_tf(std::string fromFrame, std::string toFrame);
};