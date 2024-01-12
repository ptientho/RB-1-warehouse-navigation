#pragma once
//#include "behaviortree_cpp_v3/action_node.h"
//#include "behaviortree_cpp_v3/basic_types.h"
//#include "behaviortree_cpp_v3/tree_node.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include <future>
#include <memory>
#include <string>
//#include "behaviortree_cpp_v3/blackboard.h"

using namespace std::chrono_literals;
/* @brief a service based Abstract class for BT.
    @param ServiceT type of service
*/
template <class ServiceT>
class ServiceNode : public BT::ActionNodeBase {
public:
  /*
      @brief constructor
      @param service_node_name: BT node name
      @param conf: BT node configuration
      @param service_name: service name this node creates a client for
  */
  ServiceNode(const std::string &service_node_name, const BT::NodeConfiguration &conf,
              const std::string &service_name)
      : BT::ActionNodeBase(service_node_name, conf),
        service_node_name_(service_node_name), service_name_(service_name) {

    // create a node to use
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_,
                                 node_->get_node_base_interface());

    // Get the required items from the blackboard
    // bt_loop_duration_ = config().blackboard.get("bt_")
    bt_loop_duration_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "bt_loop_duration");
    server_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    // get the wait for server timeout from blackboard. For the sake of
    // simplicity, can hardcode this later
    wait_for_service_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "wait_for_service_timeout");

    // Create a service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = node_->create_client<ServiceT>(
        service_name_, rmw_qos_profile_services_default,callback_group_);
    // Make a request to a service
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is there before running
    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\"%s\" service server not available after waiting for 1 s",
                   service_node_name.c_str());
      throw std::runtime_error(std::string("Service server %s not available",
                                           service_node_name.c_str()));
    }
  }

  ServiceNode() = delete;
  virtual ~ServiceNode() {}

  /*
      @brief a node in BT which accepts parameters must provide a providedPorts
     and call providedBasicPorts in it.
      @param additional ports to add to BT port list
      @return BT::PortsList containing basic portsalong with node-specific ports
  */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {

    BT::PortsList basic = {
        BT::InputPort<std::string>("service_name",
                                   "please_set_service_name_in_BT_Node"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};

    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
      @brief the main override required by BT service
      @return BT::NodeStatus of tick execution

  */
  BT::NodeStatus tick() override {

    // check if should send request
    if (!request_sent_) {
      // if request is not sent, the request can be sent.
      should_send_request_ = true;
      // call on_tick()
      on_tick();

      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }

    return check_future();
  }

  /**
      @brief another override required by BT service
  */
  void halt() override {

    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
    @brief user-defined function for operation on tick. Service request with
     information if necessary

  */
  virtual void on_tick() {}

  /**
      @brief user-defined function upon successful. Could put a value on the
     blackboard.
      @param response can be used to get the result of the service call in the
     BT Node.
      @return BT::NodeStatus Returns SUCCESS by default, user may override to
     return another value
  */
  virtual BT::NodeStatus
  on_completion(std::shared_ptr<typename ServiceT::Response> resp) {

    return BT::NodeStatus::SUCCESS;
  }

  /**
      @brief check the future and decide the status of BT
      @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE
     otherwise
  */
  virtual BT::NodeStatus check_future() {

    // check remaining time is within server_timeout
    auto elapsed = (node_->now() - sent_time_)
                       .template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;
    // return status if within remaining time > 0
    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout =
          remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_)
                      .template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }
    // else send failure status
    RCLCPP_WARN(node_->get_logger(),
                "Node timed out while executing service call to %s.",
                service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
      @brief Function to perform some user-defined operation after a timeout
     waiting for a result that hasn't been received yet
  */
  virtual void on_wait_for_result() {}

protected:
  std::string service_node_name_;
  std::string service_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // node used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // timeout value while to use in the tick loop while waiting for a result from
  // the server
  std::chrono::milliseconds server_timeout_;

  // timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // timeout waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_ = false;
  rclcpp::Time sent_time_;

  // can be set in on_tick or on_wait_for_result to indicate if a request should
  // be sent.
  bool should_send_request_;
};