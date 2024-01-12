#include "rb1_autonomy/navigation_behavior.h"
//#include "behaviortree_cpp_v3/basic_types.h"
//#include "behaviortree_cpp/basic_types.h"
//#include "behaviortree_cpp_v3/basic_types.h"
#include "rb1_autonomy/action_node.h"
#include "rb1_autonomy/autonomy.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include <functional>
#include <string>
#include <vector>

/*
GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {

  this->action_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList GoToPose::providedPorts() {

  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart() {
  // get the key corresponding to "loc"
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  // read location file to node parameter
  const std::string location_file =
      node_->get_parameter("location_file").as_string();
  // read YAML file
  YAML::Node locations = YAML::LoadFile(location_file);
  // get location based on loc key. return vector of [x,y,theta]
  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // set up action client to send goal
  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->get_clock()->now();
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  this->action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_->get_logger(), "Send goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning() {
  if (done_flag_) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded\n", this->name());
    return BT::NodeStatus::SUCCESS;
  } else {

    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::nav_to_pose_callback(
    const GoalHandleNav::WrappedResult &result) {
  done_flag_ = true;
  //if (result.result) {
  //}
}
*/

/* Old version of GoToPoseActionClient
GoToPoseActionClient::GoToPoseActionClient(const std::string &xml_tag_name,
                                           const std::string &action_name,
                                           const BT::NodeConfiguration &conf,
                                           const rclcpp::Node::SharedPtr node)
    : ActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
                                                    conf, node),
      node_(node) {}

void GoToPoseActionClient::on_tick() {

  // get node from blackboard
  // node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

  // get the key corresponding to "loc"
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  // read location file to node parameter
  const std::string location_file =
      node_->get_parameter("location_file").as_string();
  // read YAML file
  YAML::Node locations = YAML::LoadFile(location_file);
  // get location based on loc key. return vector of [x,y,theta]
  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // build new goal based on goal_ variable
  auto goal_ = nav2_msgs::action::NavigateToPose::Goal();
  goal_.pose.header.frame_id = "map";
  goal_.pose.header.stamp = node_->get_clock()->now();
  goal_.pose.pose.position.x = pose[0];
  goal_.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize();
  goal_.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node_->get_logger(), "Goal message created----> [x: %f], [y: %f]",
              goal_.pose.pose.position.x, goal_.pose.pose.position.y);
}

void GoToPoseActionClient::on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
        feedback) {

  // publish feedback
  RCLCPP_INFO(node_->get_logger(), "Remaining Distance: %f",
              feedback->distance_remaining);
}
*/

/* New version of GoToPoseActionClient using behaviortree_ros2 pkg*/
bool GoToPoseActionClient::setGoal(Goal &goal) {

  // get the key corresponding to "loc"
  auto loc = getInput<std::string>("loc");
  // read location file to node parameter
  const std::string location_file =
      node_->get_parameter("location_file").as_string();

  // const std::string location_file =
  //     node_->get_parameter("location_file").as_string();
  //  read YAML file
  YAML::Node locations = YAML::LoadFile(location_file);
  // get location based on loc key. return vector of [x,y,theta]
  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // build new goal based on goal_ variable
  auto goal_ = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = pose[0];
  goal.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize();
  goal.pose.pose.orientation = tf2::toMsg(q);
  return true;
}

BT::NodeStatus GoToPoseActionClient::onResultReceived(const WrappedResult &wr) {

    RCLCPP_INFO(node_->get_logger(), "%s: Result Received.", name().c_str());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoToPoseActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback) {

    RCLCPP_INFO(node_->get_logger(), "%s: Distance Remaining: %f", name().c_str(), feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;

}