#pragma once
#include "geometry_msgs/msg/detail/polygon__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "shelf_attach_msg/srv/attach_shelf.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

class AttachShelfServer : public rclcpp::Node {

public:
  using AttachShelf = shelf_attach_msg::srv::AttachShelf;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using CmdVel = geometry_msgs::msg::Twist;
  using Elevator = std_msgs::msg::String;
  using Footprint = geometry_msgs::msg::Polygon;
  using ClientMsg = rcl_interfaces::srv::SetParameters;
  AttachShelfServer();

private:
  rclcpp::Service<AttachShelf>::SharedPtr srv_;
  void service_callback(const std::shared_ptr<AttachShelf::Request> req,
                        const std::shared_ptr<AttachShelf::Response> res);
  void move_to_front_shelf_real(const float &front_offset,
                                const float &front_speed,
                                const float &turn_speed);
  void move_to_front_shelf(const float &front_offset, const float &front_speed,
                           const float &turn_speed);

  void attach_shelf(const double& center_dist);
  void set_params();
  void publish_shelf_frame(const geometry_msgs::msg::TransformStamped &pose);

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
  rclcpp::Client<ClientMsg>::SharedPtr param_pub_;
  rclcpp::Client<ClientMsg>::SharedPtr param_pub2_;
  rclcpp::Client<ClientMsg>::SharedPtr param_pub3_;
  // tf broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
};
