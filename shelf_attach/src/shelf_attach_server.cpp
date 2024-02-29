#include "shelf_attach/shelf_attach_server.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
AttachShelfServer::AttachShelfServer() : rclcpp::Node("shelf_attach_node") {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);

  // Initialize parameters
  initializeParameters();
  // Setup TF
  setupTF();
  // Create publishers
  createPublishers();
  // Create service
  createAttachShelfService();
  RCLCPP_INFO(this->get_logger(), "Initialized attach_shelf service.");
}

void AttachShelfServer::initializeParameters() {
  // Declare parameters with default values
  declare_parameter<bool>("activate_elevator", false);
  declare_parameter<bool>("real_robot", false);
  declare_parameter<float>("attach_velocity", 0.2);
  declare_parameter<std::string>("from_frame", "robot_base_link");
  declare_parameter<std::string>("to_frame", "front_shelf");
  declare_parameter<std::string>("cart_frame", "cart");
  declare_parameter<float>("center_shelf", 0.0);
  declare_parameter<float>("cart_offset", 0.0);
}

void AttachShelfServer::setupTF() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_success_ = false;
}

void AttachShelfServer::createPublishers() {
  vel_pub_ = create_publisher<CmdVel>("/cmd_vel", 10);
  lift_pub_ = create_publisher<Elevator>("/elevator_up", 10);
  foot_pub_glob_ = create_publisher<Footprint>("/global_costmap/footprint", 10);
  foot_pub_local_ = create_publisher<Footprint>("/local_costmap/footprint", 10);
  param_pub_ = create_client<ClientMsg>("/controller_server/set_parameters");
  param_pub2_ =
      create_client<ClientMsg>("/global_costmap/global_costmap/set_parameters");
  param_pub3_ =
      create_client<ClientMsg>("/local_costmap/local_costmap/set_parameters");
}

void AttachShelfServer::createAttachShelfService() {
  srv_ = create_service<AttachShelf>(
      "/attach_shelf", std::bind(&AttachShelfServer::service_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void AttachShelfServer::service_callback(
    const std::shared_ptr<AttachShelf::Request> req,
    const std::shared_ptr<AttachShelf::Response> res) {

  // Extract parameters from request
  float front_distance = req->front_distance;
  bool is_attach_shelf = req->attach_shelf;
  RCLCPP_INFO(
      this->get_logger(),
      "Starting /attach_shelf service. Robot is moving to the front shelf.");

  // Check if the robot is a real robot
  bool real_robot = this->get_parameter("real_robot").as_bool();
  if (real_robot) {
    auto cart_offset = this->get_parameter("cart_offset").as_double();
    auto cart_frame = this->get_parameter("cart_frame").as_string();
    auto to_frame = this->get_parameter("to_frame").as_string();
    // Publish "temp_cart" from "robot_cart_laser"
    auto pose = geometry_msgs::msg::TransformStamped();
    pose.header.frame_id = "robot_cart_laser";
    pose.header.stamp = get_clock()->now();
    pose.child_frame_id = to_frame;
    pose.transform.translation.x =
        (-1) * cart_offset; // shift frame origin upfront
    pose.transform.translation.y = 0.0;
    pose.transform.translation.z = 0.0;
    pose.transform.rotation.x = 0.0;
    pose.transform.rotation.y = 0.0;
    pose.transform.rotation.z = 0.0;
    pose.transform.rotation.w = 1.0;

    publishShelfFrame(pose);
    // Delay to wait for published temp_cart
    usleep(1000000);
    geometry_msgs::msg::PoseStamped map_tf =
        getTransform("map", "robot_cart_laser");
    // Publish static tf base on map frame
    pose = geometry_msgs::msg::TransformStamped();
    pose.header.frame_id = "map";
    pose.header.stamp = get_clock()->now();
    pose.child_frame_id = cart_frame;
    pose.transform.translation.x = map_tf.pose.position.x;
    pose.transform.translation.y = map_tf.pose.position.y;
    pose.transform.translation.z = map_tf.pose.position.z;
    pose.transform.rotation.x = map_tf.pose.orientation.x;
    pose.transform.rotation.y = map_tf.pose.orientation.y;
    pose.transform.rotation.z = map_tf.pose.orientation.z;
    pose.transform.rotation.w = map_tf.pose.orientation.w;

    publishShelfFrame(pose);
    usleep(1000000);
    // Control shelf attach
    move_to_front_shelf_real(front_distance, 0.03, 0.03); // 0.03, 0.03
  } else {
    move_to_front_shelf(front_distance, 0.08, 0.2);
  }

  if (!is_attach_shelf) {
    res->is_success = false;
  } else {
    // Move undershelf
    auto center_shelf = this->get_parameter("center_shelf").as_double();
    attachShelf(center_shelf);
    setParameters();
    res->is_success = true;
  }
}

void AttachShelfServer::stopRobot() {
  CmdVel msg = geometry_msgs::msg::Twist();
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;
  vel_pub_->publish(msg);
}

void AttachShelfServer::move_to_front_shelf_real(const float &front_offset,
                                                 const float &front_speed,
                                                 const float &turn_speed) {
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("to_frame").as_string();
  updateTFParameters(fromFrame, toFrame);
  orientRobotHeadReal(front_offset, front_speed, turn_speed);
  stopRobot();
  RCLCPP_INFO(this->get_logger(), "Oriented to shelf done.");
  alignToShelf(turn_speed);
  stopRobot();
  RCLCPP_INFO(this->get_logger(), "Aligned to shelf done.");
}

void AttachShelfServer::move_to_front_shelf(const float &front_offset,
                                            const float &front_speed,
                                            const float &turn_speed) {
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("to_frame").as_string();
  updateTFParameters(fromFrame, toFrame);
  orientRobotHeadSim(front_offset, front_speed, turn_speed);
  stopRobot();
  RCLCPP_INFO(this->get_logger(), "Oriented to shelf done.");
}

void AttachShelfServer::updateTFParameters(const std::string &fromFrame,
                                           const std::string &toFrame) {

  // Update control variables
  tf_robot_shelf = getTransform(fromFrame, toFrame);
  diff_x = tf_robot_shelf.pose.position.x;
  diff_y = tf_robot_shelf.pose.position.y;
  alpha = abs(M_PI / 2 - atan2(diff_x, diff_y));
  // Orientation
  double roll, pitch;
  tf2::Quaternion q;
  tf2::fromMsg(tf_robot_shelf.pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void AttachShelfServer::orientRobotHeadReal(const float &front_offset,
                                            const float &front_speed,
                                            const float &turn_speed) {
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("to_frame").as_string();
  rclcpp::Rate loop_rate(10);
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  // Control loop for orientation
  while (diff_x >= front_offset) {
    // Update variables
    updateTFParameters(fromFrame, toFrame);
    if (!tf_success_){
        break;
    }
    vel_msg.linear.x = front_speed;

    if (alpha >= 0.0) {
      if (diff_y >= 0) {
        vel_msg.angular.z = turn_speed;

      } else {
        vel_msg.angular.z = (-1) * turn_speed;
      }
    } else {
      vel_msg.angular.z = 0.0;
    }

    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }
}

void AttachShelfServer::orientRobotHeadSim(const float &front_offset,
                                           const float &front_speed,
                                           const float &turn_speed) {
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("to_frame").as_string();
  rclcpp::Rate loop_rate(10);
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  // Control loop for orientation
  while (abs(diff_x) >= front_offset) {
    // Update variables
    updateTFParameters(fromFrame, toFrame);
    if (!tf_success_){
        break;
    }
    vel_msg.linear.x = front_speed;
    if (alpha > 0.01) {
      if (diff_y >= 0) {
        vel_msg.angular.z = turn_speed;
      } else {
        vel_msg.angular.z = (-1) * turn_speed;
      }
    } else {
      vel_msg.angular.z = 0.0;
    }
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }
}

void AttachShelfServer::alignToShelf(const float &turn_speed) {
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("to_frame").as_string();
  rclcpp::Rate loop_rate(10);
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  // Control loop for shelf alignment
  while (abs(yaw) > 0.01) {
    // Update variables
    updateTFParameters(fromFrame, toFrame);
    if (!tf_success_){
        break;
    }
    if (yaw >= 0) {
      vel_msg.angular.z = turn_speed;
    } else {
      vel_msg.angular.z = (-1) * turn_speed;
    }
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }
}

void AttachShelfServer::attachShelf(const double &center_dist) {

  CmdVel vel_msg;
  bool elevator_up;
  float front_vel;
  this->get_parameter("activate_elevator", elevator_up);
  this->get_parameter("attach_velocity", front_vel);
  std::string fromFrame = this->get_parameter("from_frame").as_string();
  std::string toFrame = this->get_parameter("cart_frame").as_string();

  rclcpp::Rate loop_rate(10);
  updateTFParameters(fromFrame, toFrame);

  // Control loop for move to center shelf
  while (diff_x > (-1) * center_dist) {
    updateTFParameters(fromFrame, toFrame);
    if (!tf_success_){
        break;
    }
    vel_msg.linear.x = front_vel;
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }
  stopRobot();
  RCLCPP_INFO(this->get_logger(), "Attached to shelf done.");

  // Activate lift
  if (elevator_up && tf_success_) {
    Elevator lift_msg = std_msgs::msg::String();
    lift_pub_->publish(lift_msg);
    std::this_thread::sleep_for(7s);
  }
}

PoseStamped AttachShelfServer::getTransform(const std::string &fromFrame,
                                            const std::string &toFrame) {

  geometry_msgs::msg::TransformStamped t;
  auto shelf_pose = geometry_msgs::msg::PoseStamped();

  try {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {

    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
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

void AttachShelfServer::setParameters() {

  // Set global and local footprint
  Footprint footprint;
  // Initialize each Point32 in the points array separately
  geometry_msgs::msg::Point32 point1, point2, point3, point4;

  point1.x = 0.5;
  point1.y = 0.5;
  point1.z = 0.0;

  point2.x = 0.5;
  point2.y = -0.5;
  point2.z = 0.0;

  point3.x = -0.5;
  point3.y = -0.5;
  point3.z = 0.0;

  point4.x = -0.5;
  point4.y = 0.5;
  point4.z = 0.0;

  // Fill the points array
  footprint.points.push_back(point1);
  footprint.points.push_back(point2);
  footprint.points.push_back(point3);
  footprint.points.push_back(point4);

  foot_pub_glob_->publish(footprint);
  foot_pub_local_->publish(footprint);

  // Set ObstacleFoorprint critics
  auto request = std::make_shared<ClientMsg::Request>();
  // Change inflation layer parameters
  auto request2 = std::make_shared<ClientMsg::Request>();
  auto request3 = std::make_shared<ClientMsg::Request>();
  //////////////////////////////////////////////////////////
  rcl_interfaces::msg::ParameterValue val;
  val.type = 9;
  val.string_array_value = {"RotateToGoal", "Oscillation", "ObstacleFootprint",
                            "GoalAlign",    "PathAlign",   "PathDist",
                            "GoalDist"};

  rcl_interfaces::msg::Parameter param;
  param.name = "FollowPath.critics";
  param.value = val;

  request->parameters.push_back(param);
  ///////////////////////////////////////////////
  // Set inflation_radius
  val.type = 3;
  val.double_value = 0.4;
  param.name = "inflation_layer.inflation_radius";
  param.value = val;

  request2->parameters.push_back(param);
  request3->parameters.push_back(param);
  ////////////////////////////////////////////
  // Set cost_scaling_factor
  val.type = 3;
  val.double_value = 1.8;
  param.name = "inflation_layer.cost_scaling_factor";
  param.value = val;

  request2->parameters.push_back(param);
  request3->parameters.push_back(param);
  ////////////////////////////////////////////
  // Set plugin layers for global costmap
  val.type = 9;
  val.string_array_value = {"static_layer", "inflation_layer"};
  param.name = "plugins";
  param.value = val;

  request2->parameters.push_back(param);
  ///////////////////////////////////////////
  while (!param_pub_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto result = param_pub_->async_send_request(request);
  auto result2 = param_pub2_->async_send_request(request2);
  auto result3 = param_pub3_->async_send_request(request3);

  RCLCPP_INFO(this->get_logger(), "Footprint and inflation parameters set.");
}

void AttachShelfServer::publishShelfFrame(
    const geometry_msgs::msg::TransformStamped &pose) {

  auto t = geometry_msgs::msg::TransformStamped();
  t.header.frame_id = pose.header.frame_id;
  t.child_frame_id = pose.child_frame_id;
  t.header.stamp = pose.header.stamp;

  t.transform.translation.x = pose.transform.translation.x;
  t.transform.translation.y = pose.transform.translation.y;
  t.transform.translation.z = pose.transform.translation.z;
  t.transform.rotation.x = pose.transform.rotation.x;
  t.transform.rotation.y = pose.transform.rotation.y;
  t.transform.rotation.z = pose.transform.rotation.z;
  t.transform.rotation.w = pose.transform.rotation.w;

  RCLCPP_INFO(this->get_logger(), "%s frame published",
              t.child_frame_id.c_str());
  tf_pub_->sendTransform(t);
}
