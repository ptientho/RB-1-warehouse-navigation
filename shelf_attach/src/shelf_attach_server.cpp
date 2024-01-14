#include "shelf_attach/shelf_attach_server.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include <chrono>
#include <functional>
#include <math.h>

using namespace std::chrono_literals;
AttachShelfServer::AttachShelfServer() : rclcpp::Node("shelf_attach_node") {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);

  this->declare_parameter<bool>("activate_elevator", true);
  this->declare_parameter<float>("attach_velocity", 0.3);

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
  this->lift_pub_ = this->create_publisher<Elevator>("/elevator_up", 10);
  this->foot_pub_glob_ =
      this->create_publisher<Footprint>("/global_costmap/footprint", 10);
  this->foot_pub_local_ =
      this->create_publisher<Footprint>("/local_costmap/footprint", 10);

  this->srv_ = this->create_service<AttachShelf>(
      "/attach_shelf", std::bind(&AttachShelfServer::service_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Initializing attach_shelf service");
}

void AttachShelfServer::service_callback(
    const std::shared_ptr<AttachShelf::Request> req,
    const std::shared_ptr<AttachShelf::Response> res) {

  RCLCPP_INFO(
      this->get_logger(),
      "Start /attach_shelf service. Robot is moving to the front shelf");
  move_to_front_shelf();
  RCLCPP_INFO(this->get_logger(), "Moving to front shelf done.");

  if (!req->attach_shelf) {
    // if attach_shelf = false -> is_success = false
    res->is_success = false;
  } else {
    // do attach_shelf()
    RCLCPP_INFO(this->get_logger(), "Robot is attaching to shelf");
    attach_shelf();
    set_params();
    res->is_success = true;
    RCLCPP_INFO(this->get_logger(), "Attaching to shelf done.");
  }
}

void AttachShelfServer::move_to_front_shelf() {
  rclcpp::Rate loop_rate(10);
  // shelf_pose is wrt map frame. get tf robot_base_link -> front_shelf
  auto tf_robot_shelf = get_tf("robot_front_laser_base_link", "front_shelf");

  // move forward distance. For the sake of simplicity, use just x distance;
  auto distance = tf_robot_shelf.pose.position.x;
  // create vel message
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  auto vx = 0.2;

  while (distance > 0.15) {
    // move forward
    vel_msg.linear.x = vx; // m/s
    vel_pub_->publish(vel_msg);

    // update distance
    tf_robot_shelf = get_tf("robot_front_laser_base_link", "front_shelf");
    distance = tf_robot_shelf.pose.position.x;
    RCLCPP_INFO(this->get_logger(), "Front shelf distance: %f", distance);
    loop_rate.sleep();
  }

  // stop robot
  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);
}

void AttachShelfServer::attach_shelf() {

  CmdVel vel_msg;
  bool elevator_up;
  float front_vel;
  this->get_parameter("activate_elevator", elevator_up);
  this->get_parameter("attach_velocity", front_vel);
  // move to center shelf
  for (int i = 0; i < 10; i++) {
    vel_msg.linear.x = front_vel;
    vel_pub_->publish(vel_msg);
    std::this_thread::sleep_for(0.5s);
  }

  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);

  // activate lift
  if (elevator_up) {
    Elevator lift_msg = std_msgs::msg::String();
    lift_pub_->publish(lift_msg);

    // set footprint
  }
}

geometry_msgs::msg::PoseStamped AttachShelfServer::get_tf(std::string fromFrame,
                                                          std::string toFrame) {

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

void AttachShelfServer::set_params() {

  // set global and local footprint
  Footprint footprint;
  // Initialize each Point32 in the points array separately
  geometry_msgs::msg::Point32 point1, point2, point3, point4;

  point1.x = 0.325;
  point1.y = 0.325;
  point1.z = 0.0;

  point2.x = 0.325;
  point2.y = -0.325;
  point2.z = 0.0;

  point3.x = -0.325;
  point3.y = -0.325;
  point3.z = 0.0;

  point4.x = -0.325;
  point4.y = 0.325;
  point4.z = 0.0;

  // Fill the points array
  footprint.points.push_back(point1);
  footprint.points.push_back(point2);
  footprint.points.push_back(point3);
  footprint.points.push_back(point4);
  
  foot_pub_glob_->publish(footprint);
  foot_pub_local_->publish(footprint);
  RCLCPP_INFO(this->get_logger(), "Footprint parameter set.");    
}
