#pragma once
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "shelf_detect_msg/srv/go_to_shelf.hpp"
#include <future>
#include <memory>
#include <string>

using namespace BT;

/* New service node */
class ShelfDetectionClient
    : public RosServiceNode<shelf_detect_msg::srv::GoToShelf> {

public:
  ShelfDetectionClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosServiceNode<shelf_detect_msg::srv::GoToShelf>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::OutputPort<bool>("find_shelf")});
  }

  bool setRequest(Request::SharedPtr &request) override;

  BT::NodeStatus
  onResponseReceived(const Response::SharedPtr &response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
  }

};
