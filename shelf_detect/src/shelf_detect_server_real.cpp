// detect shelf server
#include "shelf_detect/shelf_detect_server_real.hpp"
#include "cmath"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mutex"
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
#include <mutex>
#include <string>
#include <vector>

ShelfDetectionServerReal::ShelfDetectionServerReal()
    : rclcpp::Node("shelf_detection_real_node") {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_DEBUG);
  RCLCPP_INFO(this->get_logger(), "Initializing go_to_shelf Service");

  shelf_found = false;
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
  this->tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  shelf_found = false;

  this->timer1_ = this->create_wall_timer(
      20ms, std::bind(&ShelfDetectionServerReal::detect_shelf, this),
      timer_group_);
  RCLCPP_INFO(this->get_logger(), "Initialized go_to_shelf Service");
}

void ShelfDetectionServerReal::service_callback(
    const std::shared_ptr<GoToShelf::Request> req,
    const std::shared_ptr<GoToShelf::Response> rsp) {

  RCLCPP_INFO(this->get_logger(), "Start go_to_shelf service");

  // delay to wait for detect_shelf method
  std::this_thread::sleep_for(0.5s);
  RCLCPP_INFO(this->get_logger(), "Found Shelf: %s",
              shelf_found ? "YES" : "NO");
  std::lock_guard<std::mutex> guard(find_shelf_mutex);
  if (shelf_found) {
    // publish front_shelf frame
    publish_shelf_frame();
    // get shelf_pose
    std::this_thread::sleep_for(0.5s);
    auto pose = get_tf("map", "front_shelf");
    rsp->shelf_pose = pose;
    rsp->shelf_found = shelf_found;
    
  } else {
    rsp->shelf_pose = get_tf("map", "front_shelf");
    rsp->shelf_found = shelf_found;
  }
}

void ShelfDetectionServerReal::laser_callback(
    const std::shared_ptr<LaserScan> msg) {

  if (msg == nullptr) {
    return;
  }
  std::vector<float> intensity_list = msg->intensities;

  laser_data = msg;

  int intensity_length = intensity_list.size();
  RCLCPP_DEBUG(this->get_logger(), "INTENSITY LENGTH ==> %d", intensity_length);
  // max intensity
  auto max_pointer_ =
      std::max_element(intensity_list.begin(), intensity_list.end());
  // min intensity
  auto min_pointer_ =
      std::min_element(intensity_list.begin(), intensity_list.end());
  RCLCPP_DEBUG(this->get_logger(), "INTENSITY VALUES | MAX: %f, MIN: %f",
               *max_pointer_, *min_pointer_);
}

void ShelfDetectionServerReal::detect_shelf() {
  // intensity array
  if (laser_data == nullptr) {
    RCLCPP_INFO(this->get_logger(), "Laser data not received.");
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
  float threshold = 4500.0; // the lowest intensity difference from max value
                            // that would be considered as a detection
  // float deviation_max = abs(*max_pointer_ - diff);
  // float deviation_min = abs(*min_pointer_ - diff);
  // RCLCPP_INFO(this->get_logger(), "Deviation max: %f", *max_pointer_);
  // RCLCPP_INFO(this->get_logger(), "Deviation min: %f", *min_pointer_);
  int num_legs = 0;

  for (it = intensity_list.begin(), i = 0;
       it != intensity_list.end() && std::next(it) != intensity_list.end();
       it++, i++) {
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

  RCLCPP_DEBUG(this->get_logger(), "NUMBER OF SHELF LEG BEING DETECTED: %d",
               num_legs);
  RCLCPP_DEBUG(this->get_logger(), "LEG 1 RANGE: %f | INDEX: %d", leg1_range,
               leg1_idx_mid);
  RCLCPP_DEBUG(this->get_logger(), "LEG 2 RANGE: %f | INDEX: %d", leg2_range,
               leg2_idx_mid);

  std::lock_guard<std::mutex> guard(find_shelf_mutex);
  if (num_legs >= 2) {
    shelf_found = true;
    // reset pointer
  } else {
    shelf_found = false;
    // reset pointer
  }
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
    RCLCPP_INFO(this->get_logger(), "odom data not received");
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
