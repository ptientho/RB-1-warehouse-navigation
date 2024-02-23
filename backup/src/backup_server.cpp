#include "backup/backup_server.hpp"
#include "rclcpp/logging.hpp"
#include <string>

BackUpServer::BackUpServer() : rclcpp::Node("backup_server_node") {

  this->declare_parameter<std::string>("from_frame", "robot_base_link");
  this->declare_parameter<std::string>("to_frame", "cart");
  this->declare_parameter<double>("backup_distance", 0.0);
  this->declare_parameter<double>("tolerance", 0.0);

  this->vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->srv_ = this->create_service<BackUp>(
      "/backup", std::bind(&BackUpServer::service_callback, this,
                           std::placeholders::_1, std::placeholders::_2));

  this->tf_success_ = false;
  RCLCPP_INFO(this->get_logger(), "Initializing Backup server...");
}

Pose BackUpServer::get_tf(std::string fromFrame, std::string toFrame) {

  geometry_msgs::msg::TransformStamped t;
  auto shelf_pose = geometry_msgs::msg::PoseStamped();
  try {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {

    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                fromFrame.c_str(), toFrame.c_str(), ex.what());
    shelf_pose.header.stamp = this->get_clock()->now();
    shelf_pose.header.frame_id = "robot_base_link";
    shelf_pose.pose.position.x = 0.0;
    shelf_pose.pose.position.y = 0.0;
    shelf_pose.pose.position.z = 0.0;
    shelf_pose.pose.orientation.x = 0.0;
    shelf_pose.pose.orientation.y = 0.0;
    shelf_pose.pose.orientation.z = 0.0;
    shelf_pose.pose.orientation.w = 1.0;
    this->tf_success_ = false;
    return shelf_pose;
  }

  auto translation_pose = t.transform.translation;
  auto rotation_pose = t.transform.rotation;

  shelf_pose.pose.position.x = translation_pose.x;
  shelf_pose.pose.position.y = translation_pose.y;
  shelf_pose.pose.position.z = translation_pose.z;
  shelf_pose.pose.orientation.x = rotation_pose.x;
  shelf_pose.pose.orientation.y = rotation_pose.y;
  shelf_pose.pose.orientation.z = rotation_pose.z;
  shelf_pose.pose.orientation.w = rotation_pose.w;
  this->tf_success_ = true;
  return shelf_pose;
}

void BackUpServer::service_callback(
    const std::shared_ptr<BackUp::Request> req,
    const std::shared_ptr<BackUp::Response> res) {

  auto fromFrame = this->get_parameter("from_frame").as_string();
  auto toFrame = this->get_parameter("to_frame").as_string();
  auto backupDist = this->get_parameter("backup_distance").as_double();
  auto TOLERANCE = this->get_parameter("tolerance").as_double();

  bool is_backup = req->is_backup_call;
  if (!is_backup) {
    res->is_success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Start /backup service. Robot is moving backward");
  const float VEL = 0.05;
  CmdVel vel_msg;
  rclcpp::Rate loop_rate(10);

  Pose pose = get_tf(fromFrame, toFrame);
  float current_x = pose.pose.position.x;
  float current_y = pose.pose.position.y;

  if (!this->tf_success_) {
    res->is_success = false;
    RCLCPP_INFO(this->get_logger(), "Cannot find transform from %s to %s",
                fromFrame.c_str(), toFrame.c_str());
    return;
  }

  if (abs(current_y) >= (float)TOLERANCE) {
    res->is_success = false;
    RCLCPP_INFO(this->get_logger(),
                "Cannot backup. Offset in Y is more than tolerance %f",
                TOLERANCE);
    return;
  }

  while (current_x < (float)backupDist) {
    // update
    pose = get_tf(fromFrame, toFrame);
    current_x = pose.pose.position.x;
    current_y = pose.pose.position.y;
    vel_msg.linear.x = (-1) * VEL;
    this->vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }

  vel_msg.linear.x = 0.0;
  this->vel_pub_->publish(vel_msg);
  res->is_success = true;
}
