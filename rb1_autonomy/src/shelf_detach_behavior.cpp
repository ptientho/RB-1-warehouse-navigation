#include "rb1_autonomy/shelf_detach_behavior.h"
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

bool DetachShelfClient::setRequest(Request::SharedPtr &request) {

    getInput("detach_shelf", request->detach_shelf);
    RCLCPP_INFO(node_->get_logger(), "%s: Request sent. detach_shelf = %s", name().c_str(), request->detach_shelf ? "true" : "false");
    return true;
}

BT::NodeStatus
  DetachShelfClient::onResponseReceived(const Response::SharedPtr &response) {
  
    RCLCPP_INFO(node_->get_logger(), "%s: Response received. | detach shelf: %s", name().c_str(), response->is_success ? "yes":"no");
    setOutput("is_success", response->is_success);
    return BT::NodeStatus::SUCCESS;
  }