#include "rb1_autonomy/shelf_attach_behavior.h"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rb1_autonomy/autonomy.h"
#include "rb1_autonomy/service_node.h"
#include "rclcpp/create_client.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <string>

using namespace std::chrono_literals;

/* New servide node */

bool AttachShelfClient::setRequest(Request::SharedPtr &request) {

    getInput("attach_shelf", request->attach_shelf);
    getInput("front_distance", request->front_distance);
    RCLCPP_INFO(node_->get_logger(), "%s: Request sent. attach_shelf = %s", name().c_str(), request->attach_shelf ? "true" : "false");
    return true;
}

BT::NodeStatus
  AttachShelfClient::onResponseReceived(const Response::SharedPtr &response) {
  
    RCLCPP_INFO(node_->get_logger(), "%s: Response received. | attach shelf: %s", name().c_str(), response->is_success ? "yes":"no");
    setOutput("is_success", response->is_success);
    return BT::NodeStatus::SUCCESS;
  }