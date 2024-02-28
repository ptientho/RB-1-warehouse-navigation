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

  this->declare_parameter<float>("detach_velocity", 0.3);

  this->vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
  this->lift_pub_ = this->create_publisher<Elevator>("/elevator_down", 10);
  this->foot_pub_glob_ = this->create_client<ClientMsg>(
      "/global_costmap/global_costmap/set_parameters");
  this->foot_pub_local_ = this->create_client<ClientMsg>(
      "/local_costmap/local_costmap/set_parameters");
  this->critic_pub_ =
      this->create_client<ClientMsg>("/controller_server/set_parameters");

  this->srv_ = this->create_service<DetachShelf>(
      "/detach_shelf", std::bind(&DetachShelfServer::service_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Initializing detach_shelf service");
}

void DetachShelfServer::service_callback(
    const std::shared_ptr<DetachShelf::Request> req,
    const std::shared_ptr<DetachShelf::Response> res) {

  RCLCPP_INFO(this->get_logger(), "Start /detach_shelf service.");
  if (!req->detach_shelf) {
    res->is_success = false;
  } else {
    detach_shelf();
    set_params();
    res->is_success = true;
    RCLCPP_INFO(this->get_logger(), "Detaching to shelf done.");
  }
}

void DetachShelfServer::detach_shelf() {

  // Unloading shelf
  Elevator lift_msg = std_msgs::msg::String();
  lift_pub_->publish(lift_msg);
  rclcpp::Rate loop_rate(10);

  // Move backward to detach
  CmdVel vel_msg;
  float back_vel;
  this->get_parameter("detach_velocity", back_vel);

  for (int i = 0; i < 20; i++) {
    vel_msg.linear.x = (-1) * back_vel;
    vel_pub_->publish(vel_msg);
    loop_rate.sleep();
  }

  vel_msg.linear.x = 0.0;
  vel_pub_->publish(vel_msg);
}

void DetachShelfServer::set_params() {

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

