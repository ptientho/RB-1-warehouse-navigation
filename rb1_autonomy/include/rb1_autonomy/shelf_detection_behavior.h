#pragma once
//#include "behaviortree_cpp_v3/action_node.h"
//#include "behaviortree_cpp_v3/basic_types.h"
//#include "behaviortree_cpp_v3/tree_node.h"
//#include "behaviortree_cpp/tree_node.h"
//#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "rb1_autonomy/service_node.h"
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
//#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_ros2/bt_service_node.hpp"

/*
class ShelfDetectionClient : public BT::SyncActionNode, public rclcpp::Node {
public:
  ShelfDetectionClient(const std::string &name,
                       const BT::NodeConfiguration &conf);
  using GoToShelf = shelf_detect_msg::srv::GoToShelf;

  //rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<GoToShelf>::SharedPtr client_ptr_;

  static BT::PortsList providedPorts() {

    return {BT::InputPort<std::string>("service_name"),
            BT::OutputPort<bool>("find_shelf"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("shelf_pose",
                                                            "Shelf Pose")};
  }

  /// throws if the derived class return RUNNING.
  BT::NodeStatus tick() override;

  ~ShelfDetectionClient(){};

};
*/

using namespace BT;
/* Old service node */
/*
class ShelfDetectionClient
    : public ServiceNode<shelf_detect_msg::srv::GoToShelf> {

public:
  ShelfDetectionClient(const std::string &service_node_name,
                       const std::string &service_name,
                       const BT::NodeConfiguration &conf);

  void on_tick() override;

  BT::NodeStatus on_completion(
      std::shared_ptr<shelf_detect_msg::srv::GoToShelf::Response> resp);

  static BT::PortsList providedPorts() {

    return providedBasicPorts(
        {BT::OutputPort<bool>("find_shelf", "Whether the shelf is detected"),
         BT::OutputPort<geometry_msgs::msg::PoseStamped>(
             "shelf_pose", "Shelf pose in map frame")});
  }

  ~ShelfDetectionClient() {}
};
*/
/* New service node */
class ShelfDetectionClient
    : public RosServiceNode<shelf_detect_msg::srv::GoToShelf> {

public:
  ShelfDetectionClient(const std::string &name, const BT::NodeConfig &conf,
                       const BT::RosNodeParams &params)
      : RosServiceNode<shelf_detect_msg::srv::GoToShelf>(name, conf,
                                                             params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::OutputPort<bool>("find_shelf")});
  }

  bool setRequest(Request::SharedPtr &request) override;
  
  BT::NodeStatus
  onResponseReceived(const Response::SharedPtr &response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};