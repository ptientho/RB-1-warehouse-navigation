#pragma once
#include "backup_msg/srv/back_up.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using CmdVel = geometry_msgs::msg::Twist;
using Pose = geometry_msgs::msg::PoseStamped;
using BackUp = backup_msg::srv::BackUp;
class BackUpServer : public rclcpp::Node {
public:
  BackUpServer();

private:
  // private data
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<BackUp>::SharedPtr srv_;
  bool tf_success_;

  // private member functions
  void service_callback(const std::shared_ptr<BackUp::Request> req,
                        const std::shared_ptr<BackUp::Response> res);

  Pose getTransform(const std::string& fromFrame, const std::string& toFrame);
  Pose getDefaultPose();
  void initializeParameters();
  void initializeROSComponents();
  void handleTransformException(const std::string& fromFrame, const std::string& toFrame, const std::string& errorMsg);
};