#include "shelf_detect/shelf_detect_server.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf.hpp"
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

using namespace std::chrono_literals;
ShelfDetectionServer::ShelfDetectionServer()
    : rclcpp::Node("shelf_detection_node"), shelf_found(false),
      subGrp_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);
  // Initialize parameters
  initializeParameters();

  // Create subscribers
  createSubscribers();

  // Create publishers
  createPublishers();

  // Initialize transform objects
  setupTF();

  // Initialize shelf_detection server
  createDetectionService();
  RCLCPP_INFO(this->get_logger(), "Initialized shelf detection Service");
}

void ShelfDetectionServer::initializeParameters() {
  this->declare_parameter("front_offset_x", 0.0);
}

void ShelfDetectionServer::createSubscribers() {
  rclcpp::SubscriptionOptions group;
  group.callback_group = subGrp_;

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
}

void ShelfDetectionServer::createPublishers() {

  this->vel_pub_ = this->create_publisher<Twist>("/cmd_vel", 10);
}

void ShelfDetectionServer::setupTF() {
  this->tf_pub_dyn_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ShelfDetectionServer::createDetectionService() {
  this->srv_ = this->create_service<GoToShelf>(
      "/go_to_shelf",
      std::bind(&ShelfDetectionServer::service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, subGrp_);
}

void ShelfDetectionServer::service_callback(
    const std::shared_ptr<GoToShelf::Request> req,
    const std::shared_ptr<GoToShelf::Response> rsp) {

  RCLCPP_INFO(this->get_logger(), "Start go_to_shelf service");
  detectShelf();
  float front_offset = req->front_offset;
  auto front_range = computeFrontShelfDistance();
  float Inf = std::numeric_limits<float>::infinity();

  if (shelf_found && (front_range != Inf) && (leg1_range <= 1.5)) {

    // create front_shelf frame for moving to front shelf (dynamic)
    orientRobot(front_offset);
    this->timer1_ = this->create_wall_timer(
        10ms, std::bind(&ShelfDetectionServer::timer1_callback, this));
    std::this_thread::sleep_for(1.0s);
    this->timer1_->cancel();
    this->timer1_ = this->create_wall_timer(
        50ms, std::bind(&ShelfDetectionServer::timer3_callback, this));
    //////////////////////////////////////////////////////////////////
    // create temp_cart frame for reference to cart
    orientRobot(0.0);
    this->timer2_ = this->create_wall_timer(
        10ms, std::bind(&ShelfDetectionServer::timer2_callback, this));
    std::this_thread::sleep_for(1.0s);
    auto pose = geometry_msgs::msg::TransformStamped();
    auto tf_cart = getTransform("map", "temp_cart");
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();
    pose.child_frame_id = "cart";
    pose.transform.translation.x = tf_cart.pose.position.x;
    pose.transform.translation.y = tf_cart.pose.position.y;
    pose.transform.translation.z = tf_cart.pose.position.z;
    pose.transform.rotation.x = 0.0;
    pose.transform.rotation.y = 0.0;
    pose.transform.rotation.z = 0.0;
    pose.transform.rotation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "%s frame published",
                pose.child_frame_id.c_str());
    tf_pub_->sendTransform(pose);
    this->timer2_->cancel();
    
    rsp->shelf_pose = getTransform("map", "front_shelf");
    rsp->shelf_found = true;

  } else {
    rsp->shelf_pose = getTransform("map", "front_shelf");
    rsp->shelf_found = false;
  }
  RCLCPP_INFO(this->get_logger(), "Found Shelf: %s",
              (shelf_found && (front_range != Inf)) ? "YES" : "NO");
}

void ShelfDetectionServer::laser_callback(
    const std::shared_ptr<LaserScan> msg) {

  std::vector<float> intensity_list = msg->intensities;
  int intensity_length = intensity_list.size();

  // get front laser index
  front_laser_idx = intensity_length / 2 - 1;
  // transfer laser data to class scope
  laser_data = msg;

  RCLCPP_DEBUG(this->get_logger(), "INTENSITY LENGTH ==> %d", intensity_length);
  // max intensity
  auto max_pointer_ =
      std::max_element(intensity_list.begin(), intensity_list.end());
  // min intensity
  auto min_pointer_ =
      std::min_element(intensity_list.begin(), intensity_list.end());
  RCLCPP_DEBUG(this->get_logger(), "INTENSITY VALUES | MAX: %f, MIN: %f",
               *max_pointer_, *min_pointer_);

  RCLCPP_DEBUG(this->get_logger(), "FRONT LASER INDEX: %d", front_laser_idx);
}

void ShelfDetectionServer::detectShelf() {

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
  float threshold = 3700.0;
  int num_legs = 0;

  for (it = intensity_list.begin(), i = 0;
       it != intensity_list.end() && std::next(it) != intensity_list.end();
       it++, i++) {

    if ((*it < threshold) && (*(std::next(it)) >= threshold)) {
      num_legs += 1;
      // get leg1 range and leg2 range
      if (num_legs == 1) {
        leg1_idx_start = i;
      }
      if (num_legs == 2) {
        leg2_idx_start = i;
      }
    } else if ((num_legs == 1) && (*it >= threshold) &&
               (*(std::next(it)) < threshold)) {
      leg1_idx_end = i;
      leg1_idx_mid = leg1_idx_start + (leg1_idx_end - leg1_idx_start) / 2;
    } else if ((num_legs == 2) && (*it >= threshold) &&
               (*(std::next(it)) < threshold)) {
      leg2_idx_end = i;
      leg2_idx_mid = leg2_idx_start + (leg2_idx_end - leg2_idx_start) / 2;
    }
  }
  leg1_range = range_list[leg1_idx_mid];
  leg2_range = range_list[leg2_idx_mid];

  front_shelf_idx = leg1_idx_mid + (leg2_idx_mid - leg1_idx_mid) / 2;

  RCLCPP_DEBUG(this->get_logger(), "NUMBER OF SHELF LEG BEING DETECTED: %d",
               num_legs);
  RCLCPP_DEBUG(this->get_logger(), "LEG 1 RANGE: %f | INDEX: %d", leg1_range,
               leg1_idx_mid);
  RCLCPP_DEBUG(this->get_logger(), "LEG 2 RANGE: %f | INDEX: %d", leg2_range,
               leg2_idx_mid);
  RCLCPP_DEBUG(this->get_logger(), "FRONT SHELF INDEX: %d", front_shelf_idx);

  if (num_legs == 2) {
    shelf_found = true;
  } else {
    shelf_found = false;
  }
}

float ShelfDetectionServer::computeFrontShelfDistance() {

  float front_range = 0.0;
  if (laser_data == nullptr) {
    return front_range;
  }
  if (!shelf_found) {
    return front_range;
  }

  // setting parameters
  const float d = 0.85; // a distance between two legs (fixed)
  float angle_increment = laser_data->angle_increment;
  float gamma = (leg2_idx_mid - leg1_idx_mid) * angle_increment;
  // beta is angle between front shelf and either r1 or r2
  float sin_beta;

  if (front_laser_idx <= front_shelf_idx) {
    sin_beta = leg1_range * sin(gamma) / d;
    front_range = (d / 2) * sin_beta / sin(gamma / 2);
  } else {
    sin_beta = leg2_range * sin(gamma) / d;
    front_range = (d / 2) * sin_beta / sin(gamma / 2);
  }

  RCLCPP_DEBUG(this->get_logger(), "FRONT SHELF RANGE: %f", front_range);

  return front_range;
}

void ShelfDetectionServer::orientRobot(const float &offset) {

  RCLCPP_INFO(this->get_logger(), "Orienting to shelf....");
  rclcpp::Rate loop(50);
  Twist vel_msg;

  if (odom_data == nullptr) {
    return;
  }
  if (!shelf_found) {
    return;
  }

  float offset_rad = offset / 57;
  auto diff_idx = (int)std::ceil(offset_rad / laser_data->angle_increment);

  detectShelf();
  while (abs(front_shelf_idx - front_laser_idx) >
         abs(diff_idx - 5)) { // 125 index diff = 30 degree
    if (front_shelf_idx <= front_laser_idx) {
      // rotate ccw
      vel_msg.angular.z = 0.18;
      this->vel_pub_->publish(vel_msg);
    } else {
      // rotate cw
      vel_msg.angular.z = -0.18;
      this->vel_pub_->publish(vel_msg);
    }
    detectShelf();
    loop.sleep();
  }
  // stop rotating
  vel_msg.angular.z = 0.0;
  this->vel_pub_->publish(vel_msg);
  RCLCPP_INFO(this->get_logger(), "Orienting to shelf DONE");
}

void ShelfDetectionServer::publishShelfFrame(const std::string &parentFrame,
                                             const std::string &childFrame,
                                             const double &offset_x,
                                             bool is_static) {

  auto range = computeFrontShelfDistance();

  auto t = geometry_msgs::msg::TransformStamped();
  t.header.frame_id = parentFrame;
  t.child_frame_id = childFrame;
  t.header.stamp = odom_data->header.stamp;

  t.transform.translation.x = range - offset_x;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  if (is_static) {
    tf_pub_->sendTransform(t);
  } else {
    tf_pub_dyn_->sendTransform(t);
  }

  RCLCPP_INFO(this->get_logger(), "Shelf frame published");
}

void ShelfDetectionServer::odom_callback(const std::shared_ptr<Odom> msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(q);

  double r, p;
  mat.getRPY(r, p, robot_yaw);
  odom_data = msg;
}

PoseStamped ShelfDetectionServer::getTransform(const std::string &fromFrame,
                                               const std::string &toFrame) {

  geometry_msgs::msg::TransformStamped t;
  auto shelf_pose = geometry_msgs::msg::PoseStamped();

  try {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {

    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                 fromFrame.c_str(), toFrame.c_str(), ex.what());
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    shelf_pose.header.stamp = clock->now();
    shelf_pose.header.frame_id = "robot_base_link";
    shelf_pose.pose.position.x = 0.0;
    shelf_pose.pose.position.y = 0.0;
    shelf_pose.pose.position.z = 0.0;
    shelf_pose.pose.orientation.x = 0.0;
    shelf_pose.pose.orientation.y = 0.0;
    shelf_pose.pose.orientation.z = 0.0;
    shelf_pose.pose.orientation.w = 1.0;
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

void ShelfDetectionServer::timer1_callback() {

  double offset_x = this->get_parameter("front_offset_x").as_double();
  publishShelfFrame("robot_base_link", "front_shelf", offset_x);
}

void ShelfDetectionServer::timer2_callback() {

  publishShelfFrame("robot_base_link", "temp_cart", 0.0);
}

void ShelfDetectionServer::timer3_callback() {

  auto pose = geometry_msgs::msg::TransformStamped();
  auto tf_front_cart = getTransform("map", "front_shelf");
  pose.header.frame_id = "map";
  pose.header.stamp = this->get_clock()->now();
  pose.child_frame_id = "front_shelf";
  pose.transform.translation.x = tf_front_cart.pose.position.x;
  pose.transform.translation.y = tf_front_cart.pose.position.y;
  pose.transform.translation.z = tf_front_cart.pose.position.z;
  pose.transform.rotation.x = tf_front_cart.pose.orientation.x;
  pose.transform.rotation.y = tf_front_cart.pose.orientation.y;
  pose.transform.rotation.z = tf_front_cart.pose.orientation.z;
  pose.transform.rotation.w = tf_front_cart.pose.orientation.w;
  // RCLCPP_INFO(this->get_logger(), "%s frame published",
  //             pose.child_frame_id.c_str());
  tf_pub_dyn_->sendTransform(pose);
}
