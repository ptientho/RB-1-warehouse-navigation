#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "navigation_behavior.h"
#include "rb1_autonomy_msg/srv/tick_bt.hpp"
#include "rcl/publisher.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>
#include <string>
#include <vector>

using namespace BT;

// Forward declaration
class AutonomyEngine;

// Class to check if shelf is found
class CheckShelfFound : public ConditionNode {
public:
  CheckShelfFound(const std::string &name, const NodeConfiguration &config)
      : ConditionNode(name, config) {}

  static PortsList providedPorts() { return {InputPort<bool>("shelf_found")}; }

  NodeStatus tick() override;
};

// Class to check if shelf is attached
class CheckShelfAttached : public ConditionNode {
public:
  CheckShelfAttached(const std::string &name, const NodeConfiguration &config,
                     const rclcpp::Node::SharedPtr &node)
      : ConditionNode(name, config), nh_(node) {}

  static PortsList providedPorts() {
    return {InputPort<std::string>("shelf_attached")};
  }

  NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr nh_;
};

// Class representing the autonomy engine
class AutonomyEngine : public rclcpp::Node {
public:
  using TickBT = rb1_autonomy_msg::srv::TickBT;
  using Point = geometry_msgs::msg::PointStamped;

  explicit AutonomyEngine(const std::string &node_name);
  virtual ~AutonomyEngine() {}

  void registerNodes();
  void createBt(const std::string &xml_file);
  void createTreeNodeXML(const std::string &xml_dir);

private:
  BehaviorTreeFactory factory_;
  std::unique_ptr<Groot2Publisher> publisher_;
  rclcpp::Service<TickBT>::SharedPtr autonomy_server_;
  BT::Tree tree;
  const unsigned port = 1667;
  rclcpp::Subscription<Point>::SharedPtr point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Point clicked_point;

  void point_callback(const std::shared_ptr<Point> msg) {

    clicked_point.header = msg->header;
    clicked_point.point = msg->point;
  }
  void timer_callback();
  void service_callback(const std::shared_ptr<TickBT::Request> req,
                        const std::shared_ptr<TickBT::Response> rsp);
};
