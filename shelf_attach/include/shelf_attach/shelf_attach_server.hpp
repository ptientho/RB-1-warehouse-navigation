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

using AttachShelf = shelf_attach_msg::srv::AttachShelf;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using CmdVel = geometry_msgs::msg::Twist;
using Elevator = std_msgs::msg::String;
using Footprint = geometry_msgs::msg::Polygon;
using ClientMsg = rcl_interfaces::srv::SetParameters;

class AttachShelfServer : public rclcpp::Node {

public:
  AttachShelfServer();

private:
  // Data members
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  rclcpp::Publisher<Elevator>::SharedPtr lift_pub_;
  rclcpp::Publisher<Footprint>::SharedPtr foot_pub_glob_;
  rclcpp::Publisher<Footprint>::SharedPtr foot_pub_local_;
  rclcpp::Client<ClientMsg>::SharedPtr param_pub_;
  rclcpp::Client<ClientMsg>::SharedPtr param_pub2_;
  rclcpp::Client<ClientMsg>::SharedPtr param_pub3_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<AttachShelf>::SharedPtr srv_;
  PoseStamped tf_robot_shelf;
  bool tf_success_;
  float diff_x;
  float diff_y;
  double alpha;
  double yaw;

  // Member functions
  void initializeParameters();

  void setupTF();

  void createPublishers();

  void createAttachShelfService();

  void service_callback(const std::shared_ptr<AttachShelf::Request> req,
                        const std::shared_ptr<AttachShelf::Response> res);
  void move_to_front_shelf_real(const float &front_offset,
                                const float &front_speed,
                                const float &turn_speed);
  void move_to_front_shelf(const float &front_offset, const float &front_speed,
                           const float &turn_speed);

  void attachShelf(const double &center_dist);
  void setParameters();
  void publishShelfFrame(const geometry_msgs::msg::TransformStamped &pose);
  void updateTFParameters(const std::string &fromFrame,
                          const std::string &toFrame);
  PoseStamped getTransform(const std::string &fromFrame,
                           const std::string &toFrame);
  // orient robot to shelf for real robot
  void orientRobotHeadReal(const float &front_offset, const float &front_speed,
                           const float &turn_speed);
  // orient robot to shelf for sim robot
  void orientRobotHeadSim(const float &front_offset, const float &front_speed,
                          const float &turn_speed);

  void alignToShelf(const float &turn_speed);
  void stopRobot();
};