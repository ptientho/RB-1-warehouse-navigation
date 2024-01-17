#pragma once
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include <ratio>
#include <string>

using namespace BT;
class BackUpActionNode : public SyncActionNode {

  using CmdVel = geometry_msgs::msg::Twist;

public:
  BackUpActionNode(const std::string &name, const NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node)
      : SyncActionNode(name, config), node_(node) {

    this->vel_pub_ = node_->create_publisher<CmdVel>("/cmd_vel", 10);
  }

  virtual NodeStatus tick() override;
  // virtual void halt() override final;

  static BT::PortsList providedPorts() {
        // Define the ports provided by your node
        return {};
    }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
};