#include "rb1_autonomy/shelf_detection_behavior.h"
//#include "behaviortree_cpp_v3/action_node.h"
//#include "behaviortree_cpp_v3/actions/set_blackboard_node.h"
//#include "behaviortree_cpp_v3/basic_types.h"
//#include "behaviortree_cpp_v3/blackboard.h"
//#include "behaviortree_cpp_v3/tree_node.h"
//#include "behaviortree_cpp/basic_types.h"
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
/*
ShelfDetectionClient::ShelfDetectionClient(const std::string &name,
                                           const BT::NodeConfiguration &conf)
    : BT::SyncActionNode(name, conf), rclcpp::Node("shelf_detect_client_node") {
  // get service name from input port
  BT::Optional<std::string> optionalServiceName =
      getInput<std::string>("service_name");
  this->client_ptr_ =
      this->create_client<GoToShelf>(optionalServiceName.value());
}

BT::NodeStatus ShelfDetectionClient::tick() {
  // return failure of server is not running
  while (!client_ptr_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(),
                "service not available, waiting again...");
    return BT::NodeStatus::FAILURE;
  }

  // send request
  auto request = std::make_shared<GoToShelf::Request>();
  auto future_result = client_ptr_->async_send_request(request);
  // wait for result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
future_result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Found shelf: %s",
                future_result.get()->shelf_found ? "yes" : "no");

    auto pose = future_result.get()->shelf_pose;
    // if ((pose.pose.position.x == NAN) || (pose.pose.position.y == NAN) ||
    // (pose.pose.position.z == NAN)){
    //     RCLCPP_INFO(node_ptr_->get_logger(), "Cannot transform to front_shelf
    //     pose"); return BT::NodeStatus::FAILURE;
    // }

    // port to output
    setOutput<bool>("find_shelf", future_result.get()->shelf_found);
    setOutput<geometry_msgs::msg::PoseStamped>("shelf_pose", pose);

    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call service add_two_ints");
    return BT::NodeStatus::FAILURE;
  }
}
*/
/* Old servide node */
/*
ShelfDetectionClient::ShelfDetectionClient(const std::string &service_node_name, const std::string& service_name,
                                           const BT::NodeConfiguration &conf)
    : ServiceNode<shelf_detect_msg::srv::GoToShelf>(service_node_name, conf, service_name){}

void ShelfDetectionClient::on_tick(){

    RCLCPP_INFO(node_->get_logger(), "Shelf detect service request is being sent");

}

BT::NodeStatus ShelfDetectionClient::on_completion(std::shared_ptr<shelf_detect_msg::srv::GoToShelf::Response> resp){
    
    AutonomyEngine::blackboard_->set("find_shelf", resp->shelf_found);
    AutonomyEngine::blackboard_->set("shelf_pose", resp->shelf_pose);
    RCLCPP_INFO(node_->get_logger(), "Shelf detect done. Shelf found: %s", resp->shelf_found ? "yes" : "no");
    return BT::NodeStatus::SUCCESS;
}
*/

/* New servide node */

bool ShelfDetectionClient::setRequest(Request::SharedPtr &request) {

    return true;
}

BT::NodeStatus
  ShelfDetectionClient::onResponseReceived(const Response::SharedPtr &response) {
  
    RCLCPP_INFO(node_->get_logger(), "%s: Response received. | shelf_found: %s", name().c_str(), response->shelf_found ? "yes":"no");
    setOutput("find_shelf", response->shelf_found);
    return BT::NodeStatus::SUCCESS;
  }