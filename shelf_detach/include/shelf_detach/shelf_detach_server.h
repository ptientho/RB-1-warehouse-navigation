#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "shelf_detach_msg/srv/detach_shelf.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include "geometry_msgs/msg/pose_stamped.hpp"

using Pose = geometry_msgs::msg::PoseStamped;
class DetachShelfServer : public rclcpp::Node {
public:
  using DetachShelf = shelf_detach_msg::srv::DetachShelf;
  using CmdVel = geometry_msgs::msg::Twist;
  using Elevator = std_msgs::msg::String;
  using ClientMsg = rcl_interfaces::srv::SetParameters;
  DetachShelfServer();

private:
  // Data memners
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  rclcpp::Publisher<Elevator>::SharedPtr lift_pub_;
  rclcpp::Client<ClientMsg>::SharedPtr foot_pub_glob_;
  rclcpp::Client<ClientMsg>::SharedPtr foot_pub_local_;
  rclcpp::Client<ClientMsg>::SharedPtr critic_pub_;
  rclcpp::Service<DetachShelf>::SharedPtr srv_;

  // TF parameters
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool tf_success_;
  bool detach_success_;

  // Member functions
  void initializeParameters();
  void createPublishers();
  void initializeTFparameters();
  void createDetachShelfService();
  void detachShelf();
  void setParameters();

  void service_callback(const std::shared_ptr<DetachShelf::Request> req,
                        const std::shared_ptr<DetachShelf::Response> res);
  Pose getTransform(const std::string &fromFrame, const std::string &toFrame);
  Pose getDefaultPose();
  void handleTransformException(const std::string& fromFrame, const std::string& toFrame, const std::string& errorMsg);
};
