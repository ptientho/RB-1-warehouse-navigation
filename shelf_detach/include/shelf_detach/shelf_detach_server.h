#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "shelf_detach_msg/srv/detach_shelf.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class DetachShelfServer : public rclcpp::Node {

public:
  using DetachShelf = shelf_detach_msg::srv::DetachShelf;
  using CmdVel = geometry_msgs::msg::Twist;
  using Elevator = std_msgs::msg::String;
  using ClientMsg = rcl_interfaces::srv::SetParameters;
  DetachShelfServer();

private:
  // Data memners
  rclcpp::Publisher<CmdVel>::SharedPtr vel_pub_;
  rclcpp::Publisher<Elevator>::SharedPtr lift_pub_;
  rclcpp::Client<ClientMsg>::SharedPtr foot_pub_glob_;
  rclcpp::Client<ClientMsg>::SharedPtr foot_pub_local_;
  rclcpp::Client<ClientMsg>::SharedPtr critic_pub_;
  rclcpp::Service<DetachShelf>::SharedPtr srv_;

  // Member functions
  void service_callback(const std::shared_ptr<DetachShelf::Request> req,
                        const std::shared_ptr<DetachShelf::Response> res);
  void detach_shelf();
  void set_params();
};
