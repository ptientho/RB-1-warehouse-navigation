#include "rb1_autonomy/autonomy.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rb1_autonomy/backup_behavior.h"
#include "rb1_autonomy/navigate_to_des_behavior.h"
#include "rb1_autonomy/navigation_behavior.h"
#include "rb1_autonomy/navigation_pose_behavior.h"
#include "rb1_autonomy/shelf_attach_behavior.h"
#include "rb1_autonomy/shelf_detach_behavior.h"
#include "rb1_autonomy/shelf_detection_behavior.h"
#include "rb1_autonomy/shelf_detection_real_behavior.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
AutonomyEngine::AutonomyEngine(const std::string &node_name)
    : rclcpp::Node(node_name) {
  // declare location_file yaml file. This is done for example use.
  this->declare_parameter("location_file", "none");
  // declare directory of groot file
  this->declare_parameter("groot_file", "none");
  // declare goal destination parameter
  this->declare_parameter("goal_x", 0.0);
  this->declare_parameter("goal_y", 0.0);

  this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  // initialize des_subscriber
  this->point_sub_ = this->create_subscription<Point>(
      "/des_provider", 5,
      std::bind(&AutonomyEngine::point_callback, this, std::placeholders::_1), options);

  this->timer_ = this->create_wall_timer(100ms, std::bind(&AutonomyEngine::timer_callback, this), callback_group_);

  // initialize service server
  this->autonomy_server_ = this->create_service<TickBT>(
      "/rb1_autonomy_server",
      std::bind(&AutonomyEngine::service_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Starting RB-1 autonomy service...");
}

void AutonomyEngine::registerNodes() {
  ////////////////////////////////////////////////////////////////////////////////////////
  /*
  register shelf_detector BT using registerNodeType. Reccommend as it is
  simpler. Require RosNodeParams to serve this node and the service name
    */
  BT::RosNodeParams params;
  params.nh = shared_from_this();

  params.default_port_value = "go_to_shelf_real";
  params.server_timeout = std::chrono::milliseconds(20000);
  factory_.registerNodeType<ShelfDetectionRealClient>("ShelfDetectorReal",
                                                      params);
  params.default_port_value = "go_to_shelf";
  params.server_timeout = std::chrono::milliseconds(20000);
  factory_.registerNodeType<ShelfDetectionClient>("ShelfDetector", params);

  params.default_port_value = "navigate_to_pose";
  params.server_timeout = std::chrono::milliseconds(120000);
  factory_.registerNodeType<GoToPoseActionClient>("GoToPose", params);
  factory_.registerNodeType<GoToPose2ActionClient>("GoToPose2", params);
  factory_.registerNodeType<GoToPoseDes>("GoToDes", params);

  params.default_port_value = "attach_shelf";
  params.server_timeout = std::chrono::milliseconds(100000);
  factory_.registerNodeType<AttachShelfClient>("AttachShelf", params);

  params.default_port_value = "detach_shelf";
  params.server_timeout = std::chrono::milliseconds(100000);
  factory_.registerNodeType<DetachShelfClient>("DetachShelf", params);

  params.default_port_value = "backup";
  params.server_timeout = std::chrono::milliseconds(100000);
  factory_.registerNodeType<BackUpClient>("BackUp", params);

  BT::NodeBuilder builder1 = [](const std::string &name,
                                const NodeConfiguration &config) {
    return std::make_unique<CheckShelfFound>(name, config);
  };
  factory_.registerBuilder<CheckShelfFound>("CheckShelfFound", builder1);

  BT::NodeBuilder builder2 = [&](const std::string &name,
                                 const NodeConfiguration &config) {
    return std::make_unique<CheckShelfAttached>(name, config,
                                                shared_from_this());
  };
  factory_.registerBuilder<CheckShelfAttached>("CheckShelfAttached", builder2);
  // create BT from XML file using blackboard as data collection. Not
  // require in simple tree tree = createTreeFromFile(bt_xml_dir,
  // blackboard_);
  ///////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  std::string groot_file = this->get_parameter("groot_file").as_string();
  createTreeNodeXML(groot_file);
}

void AutonomyEngine::createBt(const std::string &xml_file) {

  const std::string bt_xml_dir =
      ament_index_cpp::get_package_share_directory("rb1_autonomy") + "/config" +
      "/" + xml_file;

  tree = factory_.createTreeFromFile(bt_xml_dir);

  // Check if there's an existing instance of Groot2Publisher
  if (this->publisher_) {
    // If yes, properly shut it down to release the port
    this->publisher_.reset();
  }
  // assign a new instance to publisher_
  this->publisher_ = std::make_unique<BT::Groot2Publisher>(tree, this->port);
}

void AutonomyEngine::createTreeNodeXML(const std::string &xml_dir) {
  /* This method has to be called after registering all custom nodes */
  // Generate TreeNodes for Groot2

  std::string xml_models = BT::writeTreeNodesModelXML(factory_);

  // Open the file for writing
  std::ofstream file(xml_dir);
  // Check if the file is open
  if (file.is_open()) {
    // Write the XML string to the file
    file << xml_models;

    // Close the file
    file.close();

    std::cout << "XML models saved to: " << xml_dir << std::endl;
  } else {
    std::cerr << "Error opening the file for writing." << std::endl;
  }
}

void AutonomyEngine::service_callback(
    const std::shared_ptr<TickBT::Request> req,
    const std::shared_ptr<TickBT::Response> rsp) {

  // get request
  std::string bt_xml = req->bt_xml_file;

  // create BT
  createBt(bt_xml);

  // tick BT using timer. This will not work with service callback since we
  // expect tree status from timer but the timer return nothing
  // timer_ =
  //  this->create_wall_timer(500ms, std::bind(&AutonomyEngine::tickBt, this));

  // loop for return value from tickBT
  auto tree_status = tree.tickOnce();

  while (tree_status == BT::NodeStatus::RUNNING) {
    RCLCPP_INFO(this->get_logger(), "BT is running");
    tree.sleep(std::chrono::milliseconds(500));
    tree_status = tree.tickOnce();
  }

  if (tree_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "All BT tasks done");
    rsp->bt_status = true;
  } else if (tree_status == BT::NodeStatus::FAILURE) {
    RCLCPP_INFO(this->get_logger(), "BT tasks failed");
    rsp->bt_status = false;
  }
}

void AutonomyEngine::timer_callback() {
  // set goal parameters to clicked point
  rclcpp::Parameter point_x("goal_x", this->clicked_point.point.x);
  rclcpp::Parameter point_y("goal_y", this->clicked_point.point.y);
  this->set_parameter(point_x);
  this->set_parameter(point_y);

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

NodeStatus CheckShelfFound::tick() {

  bool is_found;
  getInput("shelf_found", is_found);
  if (is_found) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

NodeStatus CheckShelfAttached::tick() {

  // get var name from input port
  auto attach_var = getInput<std::string>("shelf_attached");
  // read is_attached value from yaml file. We get the variable from
  // "location_file"
  const std::string yaml_file = nh_->get_parameter("location_file").as_string();
  YAML::Node yaml = YAML::LoadFile(yaml_file);
  bool is_attached = yaml[attach_var.value()].as<bool>();

  if (is_attached) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
