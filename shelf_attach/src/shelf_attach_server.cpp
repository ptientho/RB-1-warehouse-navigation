#include "shelf_attach/shelf_attach_server.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include <chrono>
#include <functional>

AttachShelfServer::AttachShelfServer() : rclcpp::Node("shelf_attach_node") {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);

  this->declare_parameter<double>("shelf_center_distance", 0.3);
  this->declare_parameter<double>("attach_velocity", 0.3);

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
  this->lift_pub_ = this->create_publisher<Elevator>("/elevator_up", 10);

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
    res->is_success = true;
    RCLCPP_INFO(this->get_logger(), "Attaching to shelf done.");
  }
}

void AttachShelfServer::move_to_front_shelf() {

  // shelf_pose is wrt map frame. get tf robot_base_link -> front_shelf
  auto tf_robot_shelf = get_tf("robot_front_laser_base_link", "front_shelf");

  // move forward distance. For the sake of simplicity, use just x distance;
  auto distance = tf_robot_shelf.pose.position.x;
  // create vel message
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  auto vx = 0.5;
  vel_msg.linear.x = vx; // m/s
  vel_pub_->publish(vel_msg);
  // delay for t = s/v
  auto delay = distance / vx;
  std::chrono::nanoseconds delay_ns(static_cast<int64_t>(delay * 1e9));
  rclcpp::sleep_for(delay_ns);
  // stop robot
  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);
}

void AttachShelfServer::attach_shelf() {

  Elevator lift_msg = std_msgs::msg::String();
  CmdVel vel_msg = geometry_msgs::msg::Twist();
  double center_distance;
  double front_vel;
  this->get_parameter("shelf_center_distance", center_distance);
  this->get_parameter("attach_velocity", front_vel);
  // move to center shelf
  vel_msg.linear.x = front_vel;
  vel_pub_->publish(vel_msg);
  auto delay =
      center_distance / front_vel; // center_distance = 0.3m, velocity = 0.3 m/s
  std::chrono::nanoseconds delay_ns(static_cast<int64_t>(delay * 1e9));
  rclcpp::sleep_for(delay_ns);
  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);

  // activate lift
  lift_pub_->publish(lift_msg);
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
