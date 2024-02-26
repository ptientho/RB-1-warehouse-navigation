#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "shelf_attach_msg/srv/attach_shelf.hpp"
#include "yaml-cpp/yaml.h"

#include <string>

using namespace BT;

class AttachShelfClient : public RosServiceNode<shelf_attach_msg::srv::AttachShelf> {
public:
    AttachShelfClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosServiceNode<shelf_attach_msg::srv::AttachShelf>(name, conf, params) {}

    static PortsList providedPorts() {
        return providedBasicPorts({
            InputPort<bool>("attach_shelf"),
            InputPort<float>("front_distance"),
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
