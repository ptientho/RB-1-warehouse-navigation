#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

#include <memory>
#include <string>

using namespace BT;

class GoToPoseDes : public RosActionNode<nav2_msgs::action::NavigateToPose> {
public:
    using Pose = geometry_msgs::msg::PoseStamped;

    GoToPoseDes(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

    static PortsList providedPorts() {
        return providedBasicPorts({ InputPort<double>("goal_degree") });
    }

    bool setGoal(Goal& goal) override;
    void onHalt() override{};
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    
    NodeStatus onFailure(ActionNodeErrorCode error) override {
        RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};
