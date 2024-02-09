

#ifndef SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_
#define SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf_real.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cstddef>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
;
class ShelfDetectionServerReal : public rclcpp::Node {
public:
  using GoToShelf = shelf_detect_msg::srv::GoToShelfReal;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Odom = nav_msgs::msg::Odometry;

  ShelfDetectionServerReal();

private:
  rclcpp::Service<GoToShelf>::SharedPtr srv_;
  rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
  rclcpp::Subscription<Odom>::SharedPtr odomSub_;
  rclcpp::CallbackGroup::SharedPtr subGrp_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  // timer use for dynamic tf publishing
  rclcpp::TimerBase::SharedPtr timer1_;
  // rclcpp::TimerBase::SharedPtr timer2_;

  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // odom data
  double robot_yaw;
  Odom::SharedPtr odom_data = nullptr;
  LaserScan::SharedPtr laser_data = nullptr;

  // leg properties
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

  /* send shelf_pose and shelf_found to client */
  void service_callback(const std::shared_ptr<GoToShelf::Request> req,
                        const std::shared_ptr<GoToShelf::Response> rsp);
  void odom_callback(const std::shared_ptr<Odom> msg);
  void laser_callback(const std::shared_ptr<LaserScan> msg);

  /* method to design algorithm for shelf detection whether it's available or
   * not */
  void detect_shelf();

  /* listen to existing robot_cart_laser frame */
  geometry_msgs::msg::PoseStamped get_tf(std::string fromFrame,
                                         std::string toFrame);

  /* publish frame */
  void publish_shelf_frame();
};

#endif // SHELF_DETECT__SHELF_DETECT_SERVER_REAL_HPP_