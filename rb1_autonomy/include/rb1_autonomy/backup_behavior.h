#pragma once
#include "backup_msg/srv/back_up.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include <memory>
#include <ratio>
#include <string>

using namespace BT;
using BackUp = backup_msg::srv::BackUp;
class BackUpClient : public RosServiceNode<BackUp> {

public:
  BackUpClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosServiceNode<BackUp>(name, conf, params) {}

  static BT::PortsList providedPorts() { return providedBasicPorts({BT::InputPort<bool>("is_backup")}); }

  bool setRequest(Request::SharedPtr &request) override;

  BT::NodeStatus
  onResponseReceived(const Response::SharedPtr &response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) {
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};