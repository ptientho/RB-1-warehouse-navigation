#include "rb1_autonomy/autonomy.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rb1_autonomy/backup_behavior.h"
#include "rb1_autonomy/navigation_behavior.h"
#include "rb1_autonomy/shelf_attach_behavior.h"
#include "rb1_autonomy/shelf_detach_behavior.h"
#include "rb1_autonomy/shelf_detection_behavior.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <functional>
#include <memory>

/*
const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("rb1_autonomy") + "/config";
using namespace std::chrono_literals;
AutonomyNode::AutonomyNode(const std::string &node_name) : Node(node_name) {

  this->declare_parameter("location_file", "none");
  //this->group_ =
this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  RCLCPP_INFO(this->get_logger(), "Autonomy Init Done..");
}

void AutonomyNode::setUp() {
  // initial BT
  create_bt();

  timer_ =
      this->create_wall_timer(500ms, std::bind(&AutonomyNode::tick_bt, this));
  // tick_bt();
}

void AutonomyNode::create_bt() {
  // create BT
  BT::BehaviorTreeFactory factory;
  // GoToPose node
  BT::NodeBuilder builder = [=](const std::string &name,
                                const BT::NodeConfiguration &config) {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };

  // ShelfDetector node
  BT::NodeBuilder builder2 = [=](const std::string &name,
                                 const BT::NodeConfiguration &config) {
    return std::make_unique<ShelfDetectionClient>(name, config);
  };

  factory.registerBuilder<GoToPose>("GoToPose", builder);
  factory.registerBuilder<ShelfDetectionClient>("ShelfDetector", builder2);

  this->tree_ = factory.createTreeFromFile(bt_xml_dir +
"/bt_test_go_to_pose.xml");
}

void AutonomyNode::tick_bt() {
  // tick BT when asked
  BT::NodeStatus tree_status = this->tree_.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING) {
    RCLCPP_INFO(this->get_logger(), "Navigation is running");
    return;

  } else if (tree_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Navigation Done");
  } else if (tree_status == BT::NodeStatus::FAILURE) {

    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
  }
}
*/

using namespace std::chrono_literals;
AutonomyEngine::AutonomyEngine(const std::string &node_name)
    : rclcpp::Node(node_name) {
  // declare location_file yaml file. This is done for example use.
  this->declare_parameter("location_file", "none");
}

// create static blackboard to exchange data in BT
// BT::Blackboard::Ptr AutonomyEngine::blackboard_ = BT::Blackboard::create();

void AutonomyEngine::setUp() {

  createBt();

  timer_ =
      this->create_wall_timer(500ms, std::bind(&AutonomyEngine::tickBt, this));
}

void AutonomyEngine::createBt() {
  // import BT xml file
  const std::string bt_xml_dir =
      ament_index_cpp::get_package_share_directory("rb1_autonomy") + "/config" +
      "/bt_test_go_to_pose.xml";

  /*
    setup blackboard vaiable. This may not be necessary as it's for complex
    action call
    */
  // auto bt_loop_duration = std::chrono::milliseconds(10);
  // auto server_timeout = std::chrono::milliseconds(5000);
  // auto wait_for_service_timeout = std::chrono::milliseconds(10000);
  // blackboard_->set("bt_loop_duration", bt_loop_duration);
  // blackboard_->set("server_timeout", server_timeout);
  // blackboard_->set("wait_for_service_timeout", wait_for_service_timeout);

  /*
    register shelf_detector BT using NodeBuilder
  */
  // BT::NodeBuilder builder1 = [](const std::string &name,
  //                              const BT::NodeConfiguration &config) {
  //  return std::make_unique<ShelfDetectionClient>(name, "/go_to_shelf",
  //  config);
  //};
  // register_bt_node<ShelfDetectionClient>("ShelfDetector", builder1);

  // BT::NodeBuilder builder2 = [=](const std::string &name,
  //                                const BT::NodeConfiguration &config) {
  //   return std::make_unique<GoToPoseActionClient>(name, "/navigate_to_pose",
  //                                                 config,
  //                                                 shared_from_this());
  // };
  // register_bt_node<GoToPoseActionClient>("GoToPose", builder2);

  ////////////////////////////////////////////////////////////////////////////////////////
  /*
  register shelf_detector BT using registerNodeType. Reccommend as it is
  simpler. Require RosNodeParams to serve this node and the service name
    */
  BT::RosNodeParams params;
  params.nh = shared_from_this();
  params.default_port_value = "go_to_shelf";
  factory_.registerNodeType<ShelfDetectionClient>("ShelfDetector", params);

  params.default_port_value = "navigate_to_pose";
  factory_.registerNodeType<GoToPoseActionClient>("GoToPose", params);

  params.default_port_value = "attach_shelf";
  params.server_timeout = std::chrono::milliseconds(20000);
  factory_.registerNodeType<AttachShelfClient>("AttachShelf", params);

  params.default_port_value = "detach_shelf";
  params.server_timeout = std::chrono::milliseconds(20000);
  factory_.registerNodeType<DetachShelfClient>("DetachShelf", params);

  BT::NodeBuilder builder = [=](const std::string &name,
                                const BT::NodeConfiguration &config) {
    return std::make_unique<BackUpActionNode>(name, config, shared_from_this());
  };

  factory_.registerBuilder<BackUpActionNode>("Backup", builder);
  // create BT from XML file using blackboard as data collection. Not
  // require in simple tree tree = createTreeFromFile(bt_xml_dir,
  // blackboard_);

  tree = factory_.createTreeFromFile(bt_xml_dir);
}

void AutonomyEngine::tickBt() {
  // run BT
  /* user-defined callback. Not necessary */
  // std::function<void()> onLoop =
  //     std::bind(&AutonomyEngine::loop_callback, this);
  // std::function<bool()> cancelRequested =
  //     std::bind(&AutonomyEngine::cancel_callback, this);
  // BtStatus status = run(&tree, std::chrono::milliseconds(100));

  // if (status == BtStatus::SUCCEEDED) {
  //   this->timer_->cancel();
  // }
  auto tree_status = tree.tickWhileRunning();
  if (tree_status == BT::NodeStatus::RUNNING) {
    RCLCPP_INFO(this->get_logger(), "BT is running");
    return;
  } else if (tree_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "All BT tasks done");
    timer_->cancel();
    rclcpp::shutdown();
    return;
  } else if (tree_status == BT::NodeStatus::FAILURE) {
    RCLCPP_INFO(this->get_logger(), "BT tasks failed");
    return;
  }
}

/*
    The following functions are not used.

*/
/*
template <typename ClassT>
void AutonomyEngine::register_bt_node(const std::string &xml_node_name,
                                      BT::NodeBuilder builder) {

  factory_.registerBuilder<ClassT>(xml_node_name, builder);
}

BT::Tree AutonomyEngine::createTreeFromFile(const std::string &file_path,
                                            BT::Blackboard::Ptr blackboard) {

  return factory_.createTreeFromFile(file_path, blackboard);
}

BtStatus AutonomyEngine::run(BT::Tree *tree,
                             std::chrono::milliseconds loopTimeout) {

  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      // if (cancelRequested()) {
      //   tree->rootNode()->halt();
      //   return BtStatus::CANCELED;
      // }

      result = tree->tickRoot();

      if (!loopRate.sleep()) {
        RCLCPP_WARN(rclcpp::get_logger("AutonomyEngine"),
                    "Behavior Tree tick rate %0.2f was exceeded!",
                    1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("AutonomyEngine"),
                 "Behavior tree threw exception: %s. Exiting with failure.",
                 ex.what());
    return BtStatus::FAILED;
  }
  RCLCPP_INFO(rclcpp::get_logger("AutonomyEngine"), "Behavior Tree Status:
%s.", result == BT::NodeStatus::SUCCESS ? "Succeeded" : "Failed");

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED
                                             : BtStatus::FAILED;
}
*/