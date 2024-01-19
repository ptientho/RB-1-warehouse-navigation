#include "rb1_autonomy/navigation_behavior.h"
#include "rb1_autonomy/action_node.h"
#include "rb1_autonomy/autonomy.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include <functional>
#include <string>
#include <vector>

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