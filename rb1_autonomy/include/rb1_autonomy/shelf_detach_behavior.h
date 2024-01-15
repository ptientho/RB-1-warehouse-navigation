#pragma once
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "shelf_detach_msg/srv/detach_shelf.hpp"
#include <future>
#include <memory>
#include <string>
#include "behaviortree_ros2/bt_service_node.hpp"

using namespace BT;

/* New service node */
class DetachShelfClient
    : public RosServiceNode<shelf_detach_msg::srv::DetachShelf> {

public:
  DetachShelfClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosServiceNode<shelf_detach_msg::srv::DetachShelf>(name, conf,
                                                             params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<bool>("detach_shelf"),BT::OutputPort<bool>("is_success")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  
  BT::NodeStatus
  onResponseReceived(const Response::SharedPtr &response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};