#pragma once
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
#include "shelf_attach_msg/srv/attach_shelf.hpp"
#include <future>
#include <memory>
#include <string>
#include "behaviortree_ros2/bt_service_node.hpp"

using namespace BT;

/* New service node */
class AttachShelfClient
    : public RosServiceNode<shelf_attach_msg::srv::AttachShelf> {

public:
  AttachShelfClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosServiceNode<shelf_attach_msg::srv::AttachShelf>(name, conf,
                                                             params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<bool>("attach_shelf"),BT::InputPort<float>("front_distance"),BT::OutputPort<bool>("is_success")});
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