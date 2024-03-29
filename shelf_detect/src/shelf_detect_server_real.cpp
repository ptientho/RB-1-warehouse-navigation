#include "shelf_detect/shelf_detect_server_real.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "shelf_detect_msg/srv/go_to_shelf_real.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include <mutex>
#include <thread>

ShelfDetectionServerReal::ShelfDetectionServerReal()
    : rclcpp::Node("shelf_detection_real_node"), shelf_found(false),
      subGrp_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
      timer_group_(
          create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);

  // Initialize parameters
  initializeParameters();
  // Setup TF
  setupTF();
  // Create subscribers
  createSubscribers();
  // Create service
  createDetectionService();
  // Create timer callback
  createTimers();
  RCLCPP_INFO(this->get_logger(), "Initialized detect_shelf service.");
}

void ShelfDetectionServerReal::initializeParameters() {
  // Declare parameters
  this->declare_parameter("frame", "robot_cart_laser");
  this->declare_parameter("front_shelf_offset", 0.3);
}

void ShelfDetectionServerReal::setupTF() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_success_ = false;
}

void ShelfDetectionServerReal::createSubscribers() {
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = subGrp_;
  this->odomSub_ = create_subscription<Odom>(
      "/odom", 10,
      std::bind(&ShelfDetectionServerReal::odom_callback, this,
                std::placeholders::_1),
      opt);

  this->laserSub_ = this->create_subscription<LaserScan>(
      "/scan", 5,
      std::bind(&ShelfDetectionServerReal::laser_callback, this,
                std::placeholders::_1),
      opt);
}

void ShelfDetectionServerReal::createDetectionService() {
  srv_ = create_service<GoToShelf>(
      "/go_to_shelf_real",
      std::bind(&ShelfDetectionServerReal::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void ShelfDetectionServerReal::createTimers() {
  timer1_ = this->create_wall_timer(
      20ms, std::bind(&ShelfDetectionServerReal::detectShelf, this),
      timer_group_);
}

void ShelfDetectionServerReal::service_callback(
    const std::shared_ptr<GoToShelf::Request> req,
    const std::shared_ptr<GoToShelf::Response> rsp) {

  RCLCPP_INFO(this->get_logger(), "Starting go_to_shelf service");

  try {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Sleep interrupted: %s", e.what());
  }

  std::lock_guard<std::mutex> guard(find_shelf_mutex);
  bool found;
  bool tf_success;
  found = shelf_found;

  RCLCPP_INFO(this->get_logger(), "Found Shelf: %s", found ? "YES" : "NO");
  auto pose = getTransform("map", "robot_cart_laser");
  tf_success = tf_success_;
  if (found && tf_success) {
    publishShelfFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    rsp->shelf_pose = getTransform("map", "front_shelf");
  } else {
    rsp->shelf_pose = getTransform("map", "front_shelf");
  }
  rsp->shelf_found = found && tf_success;
}

void ShelfDetectionServerReal::laser_callback(
    const std::shared_ptr<LaserScan> msg) {

  if (msg == nullptr) {
    return;
  }
  laser_data = msg;
}

void ShelfDetectionServerReal::detectShelf() {

  if (laser_data == nullptr) {
    RCLCPP_INFO(this->get_logger(), "Laser data not received.");
    return;
  }
  std::vector<float> intensity_list = laser_data->intensities;
  std::vector<float> range_list = laser_data->ranges;
  std::vector<float>::iterator it;
  int i;
  auto max_pointer_ =
      std::max_element(intensity_list.begin(), intensity_list.end());
  auto min_pointer_ =
      std::min_element(intensity_list.begin(), intensity_list.end());

  // since in real robot the signal oscillates so only max intensity cannot be
  // properly used
  float threshold = 4400.0;
  int num_legs = 0;

  for (it = intensity_list.begin(), i = 0;
       it != intensity_list.end() && std::next(it) != intensity_list.end();
       it++, i++) {

    if ((*it < threshold) && (*(std::next(it)) >= threshold)) {
      num_legs += 1;
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

  RCLCPP_DEBUG(this->get_logger(), "NUMBER OF SHELF LEG BEING DETECTED: %d",
               num_legs);
  RCLCPP_DEBUG(this->get_logger(), "LEG 1 RANGE: %f | INDEX: %d", leg1_range,
               leg1_idx_mid);
  RCLCPP_DEBUG(this->get_logger(), "LEG 2 RANGE: %f | INDEX: %d", leg2_range,
               leg2_idx_mid);

  std::lock_guard<std::mutex> guard(find_shelf_mutex);
  if (num_legs >= 2) {
    shelf_found = true;
  } else {
    shelf_found = false;
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

PoseStamped ShelfDetectionServerReal::getTransform(const std::string &fromFrame,
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
    shelf_pose.header.frame_id = "robot_base_footprint";
    shelf_pose.pose.position.x = 0.0;
    shelf_pose.pose.position.y = 0.0;
    shelf_pose.pose.position.z = 0.0;
    shelf_pose.pose.orientation.x = 0.0;
    shelf_pose.pose.orientation.y = 0.0;
    shelf_pose.pose.orientation.z = 0.0;
    shelf_pose.pose.orientation.w = 1.0;
    tf_success_ = false;
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
  tf_success_ = true;
  return shelf_pose;
}

void ShelfDetectionServerReal::publishShelfFrame() {

  if (odom_data == nullptr) {
    RCLCPP_INFO(this->get_logger(), "odom data not received");
    return;
  }

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

  tf_pub_->sendTransform(t);
  RCLCPP_INFO(this->get_logger(), "Front shelf frame published");
}
