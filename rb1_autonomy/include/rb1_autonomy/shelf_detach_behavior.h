#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/logging.hpp"
#include "shelf_detach_msg/srv/detach_shelf.hpp"
#include "yaml-cpp/yaml.h"

#include <string>

using namespace BT;

class DetachShelfClient : public RosServiceNode<shelf_detach_msg::srv::DetachShelf> {
public:
    DetachShelfClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosServiceNode<shelf_detach_msg::srv::DetachShelf>(name, conf, params) {}

    static PortsList providedPorts() {
        return providedBasicPorts({
            InputPort<bool>("detach_shelf"),
            InputPort<std::string>("shelf_attached")
        });
    }

    bool setRequest(Request::SharedPtr& request) override;
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override {
        RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
        return NodeStatus::FAILURE;
    }
};
