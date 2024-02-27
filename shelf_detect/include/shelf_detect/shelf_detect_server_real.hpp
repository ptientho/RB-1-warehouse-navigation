

#ifndef SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_
#define SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "shelf_detect_msg/srv/go_to_shelf_real.hpp"
#include <memory>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;
using GoToShelf = shelf_detect_msg::srv::GoToShelfReal;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;

class ShelfDetectionServerReal : public rclcpp::Node {
public:
  ShelfDetectionServerReal();

private:
  // Data members
  rclcpp::Service<GoToShelf>::SharedPtr srv_;
  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
  rclcpp::Subscription<Odom>::SharedPtr odomSub_;
  // Callback group
  rclcpp::CallbackGroup::SharedPtr subGrp_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr timer1_;
  // TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  // Callback data
  double robot_yaw;
  Odom::SharedPtr odom_data = nullptr;
  LaserScan::SharedPtr laser_data = nullptr;

  float leg1_range = NULL;
  float leg2_range = NULL;
  float angle_diff = NULL;
  int leg1_idx_start = 0;
  int leg2_idx_start = 0;
  int leg1_idx_end = 0;
  int leg2_idx_end = 0;
  int leg1_idx_mid = 0;
  int leg2_idx_mid = 0;
  bool shelf_found;

  // mutex declaration
  std::mutex find_shelf_mutex;

  // Member functions
  void service_callback(const std::shared_ptr<GoToShelf::Request> req,
                        const std::shared_ptr<GoToShelf::Response> rsp);
  void odom_callback(const std::shared_ptr<Odom> msg);
  void laser_callback(const std::shared_ptr<LaserScan> msg);

  void detect_shelf();

  geometry_msgs::msg::PoseStamped get_tf(const std::string& fromFrame,
                                         const std::string& toFrame);

  void publish_shelf_frame();
};

#endif // SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_