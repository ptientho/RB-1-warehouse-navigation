#include "shelf_detach/shelf_detach_server.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rate.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
DetachShelfServer::DetachShelfServer() : rclcpp::Node("shelf_detach_node") {

  rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_INFO);
  // Initialize parameters
  initializeParameters();

  // Create publishers
  createPublishers();

  // TF parameters
  initializeTFparameters();

  // Create detach shelf service
  createDetachShelfService();
  RCLCPP_INFO(this->get_logger(), "Initialized detach_shelf service");
}

void DetachShelfServer::initializeParameters() {

  declare_parameter<std::string>("from_frame", "robot_base_link");
  declare_parameter<std::string>("to_frame", "cart_goal");
  declare_parameter<double>("backup_distance", 0.0);
  detach_success_ = false;
}

void DetachShelfServer::createPublishers() {

  this->vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
  this->lift_pub_ = this->create_publisher<Elevator>("/elevator_down", 10);

  this->foot_pub_glob_ = this->create_client<ClientMsg>(
      "/global_costmap/global_costmap/set_parameters");
  this->foot_pub_local_ = this->create_client<ClientMsg>(
      "/local_costmap/local_costmap/set_parameters");
  this->critic_pub_ =
      this->create_client<ClientMsg>("/controller_server/set_parameters");
}

void DetachShelfServer::initializeTFparameters() {

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_success_ = false;
}

void DetachShelfServer::createDetachShelfService() {

  this->srv_ = this->create_service<DetachShelf>(
      "/detach_shelf", std::bind(&DetachShelfServer::service_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void DetachShelfServer::service_callback(
    const std::shared_ptr<DetachShelf::Request> req,
    const std::shared_ptr<DetachShelf::Response> res) {

  RCLCPP_INFO(this->get_logger(), "Start /detach_shelf service.");
  if (!req->detach_shelf) {
    res->is_success = false;
  } else {
    detachShelf();
    res->is_success = detach_success_;
    if (res->is_success) {
      setParameters();
    }
    RCLCPP_INFO(this->get_logger(), "Detaching to shelf done.");
  }
}

void DetachShelfServer::detachShelf() {

  auto fromFrame = get_parameter("from_frame").as_string();
  auto toFrame = get_parameter("to_frame").as_string();
  auto backupDist = get_parameter("backup_distance").as_double();
  // Unloading shelf
  Elevator lift_msg = std_msgs::msg::String();
  lift_pub_->publish(lift_msg);

  // Move backward to detach
  const float VEL = 0.05;
  CmdVel vel_msg;
  rclcpp::Rate loop_rate(10);

  Pose pose = getTransform(fromFrame, toFrame);
  float current_x = pose.pose.position.x;

  if (!tf_success_) {
    detach_success_ = false;
    RCLCPP_INFO(get_logger(), "Cannot find transform from %s to %s",
                fromFrame.c_str(), toFrame.c_str());
    return;
  }

  while (current_x < static_cast<float>(backupDist)) {
    pose = getTransform(fromFrame, toFrame);
    current_x = pose.pose.position.x;
    vel_msg.linear.x = (-1) * VEL;
    vel_msg.angular.z = 0.0;
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }

  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub_->publish(vel_msg);
  detach_success_ = true;
}

void DetachShelfServer::setParameters() {

  auto request1 = std::make_shared<ClientMsg::Request>(); // for global costmap
  auto request2 = std::make_shared<ClientMsg::Request>(); // for local costmap
  auto request3 = std::make_shared<ClientMsg::Request>(); // for setting critics
  ////////////////////////////////////////////////////
  // Set robot radius
  rcl_interfaces::msg::ParameterValue val;
  val.type = 3;
  val.double_value = 0.3;

  rcl_interfaces::msg::Parameter param;
  param.name = "robot_radius";
  param.value = val;
  request1->parameters.push_back(param);
  request2->parameters.push_back(param);
  ///////////////////////////////////////////////////
  // Set critics
  val.type = 9;
  val.string_array_value = {"RotateToGoal", "Oscillation", "BaseObstacle",
                            "GoalAlign",    "PathAlign",   "PathDist",
                            "GoalDist"};

  param.name = "FollowPath.critics";
  param.value = val;
  request3->parameters.push_back(param);
  /////////////////////////////////////////////////////
  // Set plugin layers only for global costmap
  val.type = 9;
  val.string_array_value = {"static_layer", "obstacle_layer", "inflation_layer",
                            "voxel_layer"};

  param.name = "plugins";
  param.value = val;
  request1->parameters.push_back(param);
  /////////////////////////////////////////////////////
  // Set inflation_radius
  val.type = 3;
  val.double_value = 0.4;
  param.name = "inflation_layer.inflation_radius";
  param.value = val;

  request1->parameters.push_back(param);
  request2->parameters.push_back(param);
  ///////////////////////////////////////////////////
  // Set cost_scaling_factor
  val.type = 3;
  val.double_value = 2.7;
  param.name = "inflation_layer.cost_scaling_factor";
  param.value = val;

  request1->parameters.push_back(param);
  request2->parameters.push_back(param);
  ////////////////////////////////////////////
  while (!foot_pub_glob_->wait_for_service(1s) &&
         !foot_pub_local_->wait_for_service(1s) &&
         !critic_pub_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto glob_rsp = foot_pub_glob_->async_send_request(request1);
  auto local_rsp = foot_pub_local_->async_send_request(request2);
  auto critic_rsp = critic_pub_->async_send_request(request3);
  RCLCPP_INFO(this->get_logger(), "Robot radius  and critic parameters set.");
}

Pose DetachShelfServer::getDefaultPose() {
  Pose pose;
  pose.header.stamp = get_clock()->now();
  pose.header.frame_id = "robot_base_link";
  pose.pose.orientation.w = 1.0;
  return pose;
}

void DetachShelfServer::handleTransformException(const std::string &fromFrame,
                                                 const std::string &toFrame,
                                                 const std::string &errorMsg) {
  RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
              fromFrame.c_str(), toFrame.c_str(), errorMsg.c_str());
  tf_success_ = false;
}

Pose DetachShelfServer::getTransform(const std::string &fromFrame,
                                     const std::string &toFrame) {
  geometry_msgs::msg::TransformStamped transform;
  auto pose = geometry_msgs::msg::PoseStamped();

  try {
    transform =
        tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    handleTransformException(fromFrame, toFrame, ex.what());
    return getDefaultPose();
  }

  auto translation = transform.transform.translation;
  auto rotation = transform.transform.rotation;

  pose.pose.position.x = translation.x;
  pose.pose.position.y = translation.y;
  pose.pose.position.z = translation.z;
  pose.pose.orientation.x = rotation.x;
  pose.pose.orientation.y = rotation.y;
  pose.pose.orientation.z = rotation.z;
  pose.pose.orientation.w = rotation.w;

  tf_success_ = true;
  return pose;
}
