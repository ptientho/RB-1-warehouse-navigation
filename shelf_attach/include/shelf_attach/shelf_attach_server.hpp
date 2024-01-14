#pragma once
#include "geometry_msgs/msg/detail/polygon__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "shelf_attach_msg/srv/attach_shelf.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

class AttachShelfServer : public rclcpp::Node {

public:
  using AttachShelf = shelf_attach_msg::srv::AttachShelf;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using CmdVel = geometry_msgs::msg::Twist;
  using Elevator = std_msgs::msg::String;
  using Footprint = geometry_msgs::msg::Polygon;
  AttachShelfServer();

private:
  rclcpp::Service<AttachShelf>::SharedPtr srv_;
  void service_callback(const std::shared_ptr<AttachShelf::Request> req,
                        const std::shared_ptr<AttachShelf::Response> res);
  void move_to_front_shelf();
  void attach_shelf();
  void set_params();

  // listener to tf frame
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::PoseStamped get_tf(std::string fromFrame,
                                         std::string toFrame);

  // publisher
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  rclcpp::Publisher<Elevator>::SharedPtr lift_pub_;
  rclcpp::Publisher<Footprint>::SharedPtr foot_pub_glob_;
  rclcpp::Publisher<Footprint>::SharedPtr foot_pub_local_;
};
