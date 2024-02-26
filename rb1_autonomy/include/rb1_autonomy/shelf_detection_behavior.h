#pragma once

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "shelf_detect_msg/srv/go_to_shelf.hpp"

#include <string>

using namespace BT;

class ShelfDetectionClient : public RosServiceNode<shelf_detect_msg::srv::GoToShelf> {
public:
    ShelfDetectionClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosServiceNode<shelf_detect_msg::srv::GoToShelf>(name, conf, params) {}

    static PortsList providedPorts() {
        return providedBasicPorts({
            InputPort<float>("front_offset"),
            OutputPort<geometry_msgs::msg::PoseStamped>("shelf_pose"),
            OutputPort<bool>("find_shelf")
        });
    }

    bool setRequest(Request::SharedPtr& request) override;
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override {
        RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
        return NodeStatus::FAILURE;
    }
};
