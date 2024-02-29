

#ifndef SHELF_DETECT__SHELF_DETECT_SERVER_HPP_
#define SHELF_DETECT__SHELF_DETECT_SERVER_HPP_

#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cstddef>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using GoToShelf = shelf_detect_msg::srv::GoToShelf;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class ShelfDetectionServer : public rclcpp::Node {
public:
  ShelfDetectionServer();

private:
  // Data members
  rclcpp::Service<GoToShelf>::SharedPtr srv_;
  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
  rclcpp::Subscription<Odom>::SharedPtr odomSub_;
  rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
  rclcpp::CallbackGroup::SharedPtr subGrp_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;

  // TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_dyn_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;

  // Callback data
  double robot_yaw;
  Odom::SharedPtr odom_data = nullptr;
  LaserScan::SharedPtr laser_data = nullptr;

  // leg and front shelf properties
  bool shelf_found;
  float leg1_range = NULL;
  float leg2_range = NULL;
  float angle_diff = NULL;
  int leg1_idx_start;
  int leg2_idx_start;
  int leg1_idx_end;
  int leg2_idx_end;
  int leg1_idx_mid;
  int leg2_idx_mid;
  int front_laser_idx;
  int front_shelf_idx;

  // Member functions
  void initializeParameters();
  void setupTF();
  void createSubscribers();
  void createPublishers();
  void createDetectionService();
  
  void detectShelf();
  float computeFrontShelfDistance();
  void orientRobot(const float &offset);
  void publishShelfFrame(const std::string &parentFrame,
                         const std::string &childFrame, const double &offset_x,
                         bool is_static = false);
  PoseStamped getTransform(const std::string &fromFrame,
                           const std::string &toFrame);

  void service_callback(const std::shared_ptr<GoToShelf::Request> req,
                        const std::shared_ptr<GoToShelf::Response> rsp);
  void odom_callback(const std::shared_ptr<Odom> msg);
  void laser_callback(const std::shared_ptr<LaserScan> msg);
  void timer1_callback();
  void timer2_callback();
  void timer3_callback();
  
};

#endif // SHELF_DETECT__SHELF_DETECT_SERVER_HPP_