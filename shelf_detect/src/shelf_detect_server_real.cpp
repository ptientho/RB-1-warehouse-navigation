// detect shelf server
#include "shelf_detect/shelf_detect_server_real.hpp"
#include "cmath"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf_real.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

ShelfDetectionServerReal::ShelfDetectionServerReal()
    : rclcpp::Node("shelf_detection_real_node") {
  using GoToShelf = shelf_detect_msg::srv::GoToShelfReal;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Odom = nav_msgs::msg::Odometry;

  //rcutils_logging_set_logger_level(this->get_logger().get_name(),
  //                                  RCUTILS_LOG_SEVERITY_DEBUG);
  RCLCPP_INFO(this->get_logger(), "Initializing go_to_shelf Service");
  // parameter frame to get tf from map
  this->declare_parameter("frame", "robot_cart_laser");
  this->declare_parameter("front_shelf_offset", 0.3);

  // tf listener from "front_shelf" to "map"
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // define callback group
  this->subGrp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions group;
  group.callback_group = subGrp_;

  this->timer_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // start shelf_detection server
  this->srv_ = this->create_service<GoToShelf>(
      "/go_to_shelf_real",
      std::bind(&ShelfDetectionServerReal::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  // rmw_qos_profile_services_default, srv_group_
  // start sensor callback
  this->odomSub_ = this->create_subscription<Odom>(
      "/odom", 10,
      std::bind(&ShelfDetectionServerReal::odom_callback, this,
                std::placeholders::_1),
      group);
  this->laserSub_ = this->create_subscription<LaserScan>(
      "/scan", 10,
      std::bind(&ShelfDetectionServerReal::laser_callback, this,
                std::placeholders::_1),
      group);

  // start tf objects
  this->tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  shelf_found = false;

  this->timer1_ = this->create_wall_timer(
      10ms, std::bind(&ShelfDetectionServerReal::detect_shelf, this),
      timer_group_);
  RCLCPP_INFO(this->get_logger(), "Initialized go_to_shelf Service");
}

void ShelfDetectionServerReal::service_callback(
    const std::shared_ptr<GoToShelf::Request> req,
    const std::shared_ptr<GoToShelf::Response> rsp) {

  RCLCPP_INFO(this->get_logger(), "Start go_to_shelf service");

  /* detect shelf: if shelf is found, compute front shelf distance */
  RCLCPP_INFO(this->get_logger(), "Found Shelf: %s",
              shelf_found ? "YES" : "NO");
  if (shelf_found) {
    // publish front_shelf frame
    publish_shelf_frame();
    // get shelf_pose
    std::this_thread::sleep_for(0.5s);
    rsp->shelf_pose = get_tf("map", "front_shelf");
    rsp->shelf_found = true;

  } else {
    rsp->shelf_pose = get_tf("map", "front_shelf");
    rsp->shelf_found = false;
  }
}

void ShelfDetectionServerReal::laser_callback(
    const std::shared_ptr<LaserScan> msg) {

  if (msg == nullptr) {
    return;
  }
  std::vector<float> intensity_list = msg->intensities;
  bool debug_mode = false;

  laser_data = msg;
  
  if (debug_mode) {
    int intensity_length = intensity_list.size();
    RCLCPP_DEBUG(this->get_logger(), "INTENSITY LENGTH ==> %d",
                 intensity_length);
    // max intensity
    auto max_pointer_ =
        std::max_element(intensity_list.begin(), intensity_list.end());
    // min intensity
    auto min_pointer_ =
        std::min_element(intensity_list.begin(), intensity_list.end());
    RCLCPP_DEBUG(this->get_logger(), "INTENSITY VALUES | MAX: %f, MIN: %f",
                 *max_pointer_, *min_pointer_);

    detect_shelf();
    if (shelf_found) {
      auto ranges = compute_front_shelf_distance();
    }
  }
}

void ShelfDetectionServerReal::detect_shelf() {
  // intensity array
  bool debug_mode = true;
  if (laser_data == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Laser data not received.");
    return;
  }
  std::vector<float> intensity_list = laser_data->intensities;
  std::vector<float>::iterator it;
  // range values
  std::vector<float> range_list = laser_data->ranges;

  int i;
  auto max_pointer_ =
      std::max_element(intensity_list.begin(), intensity_list.end());
  // min intensity
  auto min_pointer_ =
      std::min_element(intensity_list.begin(), intensity_list.end());

  // since in real robot the signal oscillates so only max intensity cannot be
  // properly used
  float threshold = 3700.0; // the lowest intensity difference from max value
                            // that would be considered as a detection
  // float deviation_max = abs(*max_pointer_ - diff);
  // float deviation_min = abs(*min_pointer_ - diff);
  // RCLCPP_INFO(this->get_logger(), "Deviation max: %f", *max_pointer_);
  // RCLCPP_INFO(this->get_logger(), "Deviation min: %f", *min_pointer_);
  int num_legs = 0;

  for (it = intensity_list.begin(), i = 0;
       it != intensity_list.end() && i < intensity_list.size(); it++, i++) {
    // RCLCPP_INFO(this->get_logger(), "current intensity: %f", *it);
    // RCLCPP_INFO(this->get_logger(), "next intensity: %f", *(std::next(it)));

    if ((*it < threshold) &&
        (*(std::next(it)) >=
         threshold)) { // signal 0 -> 1 // in real robot the intensity
                       // varies so CANNOT use max value to detect the leg |
                       // if ((*it == *min_pointer_) && (*(std::next(it)) ==
                       // *max_pointer_))
      num_legs += 1;
      // get leg1 range and leg2 range
      if (num_legs == 1) {
        leg1_idx_start = i;
      }
      if (num_legs == 2) {
        leg2_idx_start = i;
      }
    } else if ((num_legs == 1) && (*it >= threshold) && // *it == *max_pointer_
               (*(std::next(it)) < threshold)) {        // signal 1 -> 0
      leg1_idx_end = i;
      leg1_idx_mid = leg1_idx_start + (leg1_idx_end - leg1_idx_start) / 2;
    } else if ((num_legs == 2) && (*it >= threshold) && // *it == *max_pointer_
               (*(std::next(it)) < threshold)) {
      leg2_idx_end = i;
      leg2_idx_mid = leg2_idx_start + (leg2_idx_end - leg2_idx_start) / 2;
    }
  }
  leg1_range = range_list[leg1_idx_mid];
  leg2_range = range_list[leg2_idx_mid];

  if (debug_mode) {
    RCLCPP_DEBUG(this->get_logger(), "NUMBER OF SHELF LEG BEING DETECTED: %d",
                 num_legs);
    RCLCPP_DEBUG(this->get_logger(), "LEG 1 RANGE: %f | INDEX: %d", leg1_range,
                 leg1_idx_mid);
    RCLCPP_DEBUG(this->get_logger(), "LEG 2 RANGE: %f | INDEX: %d", leg2_range,
                 leg2_idx_mid);
    auto test_vec = compute_front_shelf_distance();
  }

  if (num_legs == 2) {
    shelf_found = true;
  } else {
    shelf_found = false;
  }
}

std::vector<float> ShelfDetectionServerReal::compute_front_shelf_distance() {
  /* use this method to compute mid shelf length */
  // get angle_increment
  std::vector<float> ranges;
  bool debug_mode = true;
  if (laser_data == nullptr) {
    ranges = {0, 0, 0, 0};
    return ranges;
  }
  float angle_increment = laser_data->angle_increment;

  // leg angles and mid angle (rad)
  float leg1_angle = leg1_idx_mid * angle_increment;
  float leg2_angle = leg2_idx_mid * angle_increment;
  float mid_angle = (laser_data->ranges.size() / 2 - 1) * angle_increment;
  // angle difference between leg range and mid range (rad)
  float angle1_diff = abs(leg1_angle - mid_angle);
  float angle2_diff = abs(leg2_angle - mid_angle);
  // leg offset and front shelf range
  float d1 = leg1_range * sin(angle1_diff);
  float d2 = leg2_range * sin(angle2_diff);
  float midleg_offset = (d1 + d2) / 2;
  float front_range = abs(leg1_range * cos(angle1_diff));

  ranges = {front_range, midleg_offset, d1, d2};
  if (debug_mode) {
    RCLCPP_INFO(this->get_logger(), "FRONT SHELF RANGE: %f", front_range);
  }

  return ranges;
}

void ShelfDetectionServerReal::odom_callback(const std::shared_ptr<Odom> msg) {

  if (msg == nullptr) {
    return;
  }
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(q);

  double r, p;
  mat.getRPY(r, p, robot_yaw);
  odom_data = msg;
}

geometry_msgs::msg::PoseStamped
ShelfDetectionServerReal::get_tf(std::string fromFrame, std::string toFrame) {

  geometry_msgs::msg::TransformStamped t;
  auto shelf_pose = geometry_msgs::msg::PoseStamped();

  try {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {

    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                 fromFrame.c_str(), toFrame.c_str(), ex.what());
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    shelf_pose.header.stamp = clock->now();
    shelf_pose.header.frame_id = "robot_base_footprint";
    shelf_pose.pose.position.x = 0.0;
    shelf_pose.pose.position.y = 0.0;
    shelf_pose.pose.position.z = 0.0;
    shelf_pose.pose.orientation.x = 0.0;
    shelf_pose.pose.orientation.y = 0.0;
    shelf_pose.pose.orientation.z = 0.0;
    shelf_pose.pose.orientation.w = 1.0;
    // rclcpp::shutdown();
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

  return shelf_pose;
}

void ShelfDetectionServerReal::publish_shelf_frame() {

  if (odom_data == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "odom data not received");
    return;
  }

  /* build tf message */
  std::string frame = this->get_parameter("frame").as_string();
  double offset = this->get_parameter("front_shelf_offset").as_double();
  auto t = geometry_msgs::msg::TransformStamped();
  t.header.frame_id = frame;
  t.child_frame_id = "front_shelf";
  t.header.stamp = odom_data->header.stamp;

  t.transform.translation.x = (-1) * (float)offset;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  /* broadcast tf topic. For static transform, broadcast once */
  // tf_pub_->sendTransform(t);
  tf_pub_->sendTransform(t);
  RCLCPP_INFO(this->get_logger(), "Front shelf frame published");
}
