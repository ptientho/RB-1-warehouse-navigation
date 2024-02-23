#include "rb1_autonomy/backup_behavior.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cmath>
#include <ratio>
#include <string>
#include <unistd.h>

using namespace std::chrono_literals;
bool BackUpClient::setRequest(Request::SharedPtr &request) {

  getInput("is_backup", request->is_backup_call);
  RCLCPP_INFO(node_->get_logger(), "%s: Request sent. backup = %s",
              name().c_str(), request->is_backup_call ? "true" : "false");
  return true;
}

BT::NodeStatus
  BackUpClient::onResponseReceived(const Response::SharedPtr &response){
  
    RCLCPP_INFO(node_->get_logger(), "%s: Response received. | backup: %s",
              name().c_str(), response->is_success ? "Success" : "Failure");

    if (response->is_success){
        return BT::NodeStatus::SUCCESS;
    }else {
        return BT::NodeStatus::FAILURE;
    }
  
  }
