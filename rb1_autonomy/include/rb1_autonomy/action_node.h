#pragma once
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
//#include "behaviortree_cpp_v3/action_node.h"
#include "rb1_autonomy/autonomy.h"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace std::chrono_literals; // NOLINT

template <class ActionT> class ActionNode : public BT::ActionNodeBase {
public:
  ActionNode(const std::string &xml_tag_name, const std::string &action_name,
             const BT::NodeConfiguration &conf,
             const rclcpp::Node::SharedPtr node)
      : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name),
        should_send_goal_(true), node_(node) {

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    bt_loop_duration_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "bt_loop_duration");
    server_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "wait_for_service_timeout");

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ =
        typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    createActionClient(action_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" ActionNode initialized",
                 xml_tag_name.c_str());
  }

  ActionNode() = delete;

  virtual ~ActionNode() {}

  void createActionClient(const std::string &action_name) {
    // Now that we have the ROS node to use, create the action client for this
    // BT action
    this->action_client_ =
        rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server",
                 action_name.c_str());

    if (!this->action_client_->wait_for_action_server(
            wait_for_service_timeout_)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\"%s\" action server not available after waiting for 1 s",
                   action_name.c_str());
      rclcpp::shutdown();
    }
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  virtual void on_tick() {}

  virtual void on_wait_for_result(
      std::shared_ptr<const typename ActionT::Feedback> /*feedback*/) {}

  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }

  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  BT::NodeStatus tick() override {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // reset the flag to send the goal or not, allowing the user the option to
      // set it in on_tick
      should_send_goal_ = true;

      // user defined function. create a goal message here. may modify
      // "should_send_goal_".
      on_tick();

      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }

      send_new_goal(); // this method should send async goal, manage goal
                       // request, feedback, and result callback

      // setup send_goal option to manage callback and set BT::NodeStatus

      RCLCPP_INFO(node_->get_logger(), "Sent new goal to server");
    }

    try {
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      if (future_goal_handle_) {
        // check timeout is within server_timeout
        auto elapsed = (node_->now() - time_goal_sent_)
                           .template to_chrono<std::chrono::milliseconds>();
        if (!(elapsed < server_timeout_)) {
          RCLCPP_WARN(node_->get_logger(),
                      "Timed out while waiting for action server to "
                      "acknowledge goal request for %s",
                      action_name_.c_str());
        } else {
          RCLCPP_INFO(node_->get_logger(),
                      "Goal accepted by server, waiting for result");
          // return BT::NodeStatus::RUNNING;
          //  The following code corresponds to the "RUNNING" loop
          if (rclcpp::ok() && !goal_result_available_) {
            // user defined callback. May modify the value of "goal_updated_"
            on_wait_for_result(feedback_);
            RCLCPP_INFO(node_->get_logger(),
                        "Waiting for feedback from bt_navigator server");
            // reset feedback to avoid stale information
            feedback_.reset();
            callback_group_executor_.spin_some();

            // check if, after invoking spin_some(), we finally received the
            // result
            if (!goal_result_available_) {
              // Yield this Action, returning RUNNING
              return BT::NodeStatus::RUNNING;
            }
          }
        }

      } else {

        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
      }

    } catch (const std::runtime_error &e) {
      if (e.what() == std::string("send_goal failed") ||
          e.what() == std::string("Goal was rejected by the action server")) {
        // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;

      } else {
        // Internal exception to propagate to the tree
        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      status = on_success();
      break;

    case rclcpp_action::ResultCode::ABORTED:
      status = on_aborted();
      break;

    case rclcpp_action::ResultCode::CANCELED:
      status = on_cancelled();
      break;

    default:
      throw std::logic_error("ActionNode::Tick: invalid status value");
    }

    goal_result_available_ = false;
    return status;
  }

  void halt() override {
    if (true) {
      /*
    auto future_result = action_client_->async_get_result(goal_handle_);
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (callback_group_executor_.spin_until_future_complete(
            future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to cancel action server for %s",
                   action_name_.c_str());
    }

    if (callback_group_executor_.spin_until_future_complete(
            future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to get result for %s in node halt!",
                   action_name_.c_str());
    }

    on_cancelled();
    */
    }

    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal() {
    // Shut the node down if it is currently running
    /*
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    if (!future_goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = this->goal_handle_.get_status()

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    */
    return false;
  }

  void send_new_goal() {
    goal_result_available_ = false;
    auto send_goal_options =
        typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&ActionNode::result_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
        std::bind(&ActionNode::feedback_callback, this, std::placeholders::_1,
                  std::placeholders::_2);

    send_goal_options.goal_response_callback = std::bind(
        &ActionNode::goal_response_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_, send_goal_options);
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle
   * has not completed. After waiting for the future to complete, this value is
   * incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False
   * otherwise
   */
  /*
 bool is_future_goal_handle_complete(std::chrono::milliseconds &elapsed) {
   auto remaining = server_timeout_ - elapsed;

   // server has already timed out, no need to sleep
   if (remaining <= std::chrono::milliseconds(0)) {
     future_goal_handle_.reset();
     return false;
   }

   auto timeout =
       remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
   auto result = callback_group_executor_.spin_until_future_complete(
       *future_goal_handle_, timeout);
   elapsed += timeout;

   if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
     future_goal_handle_.reset();
     throw std::runtime_error("send_goal failed");
   }

   if (result == rclcpp::FutureReturnCode::SUCCESS) {
     goal_handle_ = future_goal_handle_->get();
     future_goal_handle_.reset();
     if (!goal_handle_) {
       throw std::runtime_error("Goal was rejected by the action server");
     }
     return true;
   }

   return false;
 }
 */

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  // typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the action server acknowledgement when a new goal is sent
  // std::shared_ptr<std::shared_future<
  //    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  //    future_goal_handle_;

  std::shared_ptr<typename rclcpp_action::ClientGoalHandle<ActionT>>
      future_goal_handle_;
  rclcpp::Time time_goal_sent_;

  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be
  // sent.
  bool should_send_goal_;

private:
  void goal_response_callback(
      const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr
          &future) {

    future_goal_handle_ = future;
    RCLCPP_INFO(rclcpp::get_logger("AutonomyEngine"), "Get goal_handle");
  }

  void feedback_callback(
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {

    feedback_ = feedback;
  }

  void result_callback(
      const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult
          &result) {

    goal_result_available_ = true;
    result_ = result;
    // result status will be checked later
  }
};