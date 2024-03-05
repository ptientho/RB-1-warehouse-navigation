

#ifndef SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_
#define SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_

#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf_real.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;
using GoToShelf = shelf_detect_msg::srv::GoToShelfReal;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class ShelfDetectionServerReal : public rclcpp::Node {
public:
  ShelfDetectionServerReal();

private:
  // Data members
  bool shelf_found;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr subGrp_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Service<GoToShelf>::SharedPtr srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
  rclcpp::Subscription<Odom>::SharedPtr odomSub_;
  rclcpp::TimerBase::SharedPtr timer1_;

  double robot_yaw;
  Odom::SharedPtr odom_data = nullptr;
  LaserScan::SharedPtr laser_data = nullptr;

  float leg1_range = NULL;
  float leg2_range = NULL;
  float angle_diff = NULL;
  int leg1_idx_start;
  int leg2_idx_start;
  int leg1_idx_end;
  int leg2_idx_end;
  int leg1_idx_mid;
  int leg2_idx_mid;
  bool tf_success_;

  // mutex declaration
  std::mutex find_shelf_mutex;

  // Member functions
  void initializeParameters();

  void setupTF();

  void createSubscribers();

  void createDetectionService();

  void createTimers();

  void detectShelf();
  PoseStamped getTransform(const std::string &fromFrame, const std::string &toFrame);
  void publishShelfFrame();
  
  void service_callback(const std::shared_ptr<GoToShelf::Request> req,
                        const std::shared_ptr<GoToShelf::Response> rsp);
  void odom_callback(const std::shared_ptr<Odom> msg);
  void laser_callback(const std::shared_ptr<LaserScan> msg);
  
};

#endif // SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_