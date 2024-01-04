// detect shelf server
#include "shelf_detect/shelf_detect_server.hpp"
#include "cmath"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
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
#include "shelf_detect_msg/srv/go_to_shelf.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <vector>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// detect shelf
// create tf
// transform tf to map frame
// send result as goal pose

ShelfDetectionServer::ShelfDetectionServer()
    : rclcpp::Node("shelf_detection_node") {
  using GoToShelf = shelf_detect_msg::srv::GoToShelf;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Odom = nav_msgs::msg::Odometry;
  
  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);
  RCLCPP_INFO(this->get_logger(), "Initializing go_to_shelf Service");
  // define callback group
  this->subGrp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions group;
  group.callback_group = subGrp_;

  // start shelf_detection server
  this->srv_ = this->create_service<GoToShelf>(
      "/go_to_shelf", std::bind(&ShelfDetectionServer::service_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

  // start sensor callback
  this->odomSub_ = this->create_subscription<Odom>(
      "/odom", 10,
      std::bind(&ShelfDetectionServer::odom_callback, this,
                std::placeholders::_1),
      group);
  this->laserSub_ = this->create_subscription<LaserScan>(
      "/scan", 10,
      std::bind(&ShelfDetectionServer::laser_callback, this,
                std::placeholders::_1),
      group);

  // start tf objects
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ShelfDetectionServer::service_callback(
    const std::shared_ptr<GoToShelf::Request> req,
    const std::shared_ptr<GoToShelf::Response> rsp) {

  RCLCPP_INFO(this->get_logger(), "Start go_to_shelf service");

  /* detect shelf: if shelf is found, compute front shelf distance */
  //std::this_thread::sleep_for(0.5s);
  bool shelf_found = detect_shelf();
  RCLCPP_INFO(this->get_logger(), "Found Shelf: %s",
                  shelf_found ? "YES" : "NO");
  if (shelf_found) {
    publish_shelf_frame();


    rsp->shelf_pose = get_tf("front_shelf","map");
    rsp->shelf_found = true;

  }else {
    rsp->shelf_pose = get_tf("front_shelf","map");
    rsp->shelf_found = false;
  }
}

void ShelfDetectionServer::laser_callback(
    const std::shared_ptr<LaserScan> msg) {
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
    RCLCPP_INFO(this->get_logger(), "INTENSITY VALUES | MAX: %f, MIN: %f",
                *max_pointer_, *min_pointer_);
    RCLCPP_INFO(this->get_logger(), "FRONT LASER RANGE: %f",
                msg->ranges[intensity_length / 2 - 1]);
    // if (detect_shelf()) {
    //   float front_shelf_distance = compute_front_shelf_distance();
    // }
    bool shelf_found = detect_shelf();
    if (shelf_found) {
      auto ranges = compute_front_shelf_distance();
    }
  }
}

bool ShelfDetectionServer::detect_shelf() {
  // intensity array
  bool debug_mode = false;

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

  int num_legs = 0;

  for (it = intensity_list.begin(), i = 0;
       it != intensity_list.end() &&
       i < intensity_list.size();
       it++, i++) {
    if ((*it == *min_pointer_) && (*(std::next(it)) == *max_pointer_)) { // signal 0 -> 1
      num_legs += 1;
      // get leg1 range and leg2 range
      if (num_legs == 1) {
        leg1_idx_start = i;
        
      }
      if (num_legs == 2) {
        leg2_idx_start = i;
      }
    } else if ((num_legs == 1) && (*it == *max_pointer_) && (*(std::next(it)) == *min_pointer_)) { // signal 1 -> 0
        leg1_idx_end = i;
        leg1_idx_mid = leg1_idx_start + (leg1_idx_end - leg1_idx_start) / 2; 
    } else if ((num_legs == 2) && (*it == *max_pointer_) && (*(std::next(it)) == *min_pointer_)) {
        leg2_idx_end = i;
        leg2_idx_mid = leg2_idx_start + (leg2_idx_end - leg2_idx_start) / 2; 

    }
  }
  leg1_range = range_list[leg1_idx_mid];
  leg2_range = range_list[leg2_idx_mid];

  if (debug_mode) {
    RCLCPP_INFO(this->get_logger(), "NUMBER OF SHELF LEG BEING DETECTED: %d",
                num_legs);
    RCLCPP_INFO(this->get_logger(), "LEG 1 RANGE: %f | INDEX: %d", leg1_range,
                leg1_idx_mid);
    RCLCPP_INFO(this->get_logger(), "LEG 2 RANGE: %f | INDEX: %d", leg2_range,
                leg2_idx_mid);
  }

  if (num_legs == 2) {
    return true;
  } else {
    return false;
  }
}

std::vector<float> ShelfDetectionServer::compute_front_shelf_distance() {
  /* use this method to compute mid shelf length */
  // get angle_increment
  float angle_increment = laser_data->angle_increment;
  bool debug_mode = false;

  // leg angles and mid angle (rad)
  float leg1_angle = leg1_idx_mid * angle_increment;
  float leg2_angle = leg2_idx_mid * angle_increment;
  float mid_angle = (laser_data->ranges.size()/2 - 1) * angle_increment;
  // angle difference between leg range and mid range (rad)
  float angle1_diff = abs(leg1_angle - mid_angle);
  float angle2_diff = abs(leg2_angle - mid_angle);
  // leg offset and front shelf range
  float d1 = leg1_range * sin(angle1_diff);
  float d2 = leg2_range * sin(angle2_diff);
  float midleg_offset = (d1 + d2) / 2;
  float front_range = abs(leg1_range * cos(angle1_diff));

  std::vector<float> ranges = {front_range, midleg_offset, d1, d2};
  if (debug_mode) {
    RCLCPP_INFO(this->get_logger(), "FRONT SHELF RANGE: %f", front_range);
  }
  
  return ranges;
}

void ShelfDetectionServer::publish_shelf_frame() {

  // compute front shelf distance
  auto ranges =
      compute_front_shelf_distance(); // [front_range, midleg_offset, d1, d2]
                                      // where d1 + d2 = 2*midleg_offset
  auto t = geometry_msgs::msg::TransformStamped();
  t.header.frame_id = "robot_front_laser_base_link";
  t.child_frame_id = "front_shelf";
  t.header.stamp = odom_data->header.stamp;

  t.transform.translation.x = ranges[0];
  // front shelf offset in y axis of laser_base_link
  if (ranges[2] <= ranges[3]) {
    t.transform.translation.y = ranges[1] - ranges[2];
  } else {
    t.transform.translation.y = -(ranges[1] - ranges[3]);
  }
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_pub_->sendTransform(t);
}

void ShelfDetectionServer::odom_callback(const std::shared_ptr<Odom> msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(q);

  double r, p;
  mat.getRPY(r, p, robot_yaw);
  odom_data = msg;
}

geometry_msgs::msg::PoseStamped ShelfDetectionServer::get_tf(std::string fromFrame, std::string toFrame) {
    geometry_msgs::msg::TransformStamped t;
    auto shelf_pose = geometry_msgs::msg::PoseStamped();

    try {
       t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {

      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  fromFrame.c_str(), toFrame.c_str(), ex.what());
      
      shelf_pose.header.stamp = odom_data->header.stamp;
      shelf_pose.header.frame_id = "map";
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