#pragma once
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/tree_node.h"
#include "navigation_behavior.h"
#include "rcl/publisher.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include <string>
#include <vector>
/* This wrapper status is not required. */
// enum class BtStatus { SUCCEEDED, FAILED, CANCELED };
using namespace BT;
class AutonomyEngine : public rclcpp::Node {
public:
  explicit AutonomyEngine(const std::string &node_name);
  virtual ~AutonomyEngine() {}
  // static BT::Blackboard::Ptr blackboard_;

  void setUp();

  void tickBt();

  void createBt();

  /* Not neccessary */
  /*
template <typename ClassT>
void register_bt_node(const std::string &xml_node_name,
                      BT::NodeBuilder builder);

BT::Tree createTreeFromFile(const std::string &file_path,
                            BT::Blackboard::Ptr blackboard = blackboard_);

BtStatus run(BT::Tree* tree, std::chrono::milliseconds loopTimeout =
std::chrono::milliseconds(10));
  */
private:
  BT::BehaviorTreeFactory factory_;
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree;
};

class CheckShelfFound : public ConditionNode {

public:
  CheckShelfFound(const std::string &name, const NodeConfiguration &config)
      : ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("shelf_found")};
  }

  // Override the tick method with your custom implementation
  NodeStatus tick() override;
};
