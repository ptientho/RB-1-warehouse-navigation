#include "backup/backup_server.hpp"
#include "rclcpp/logging.hpp"

BackUpServer::BackUpServer() : rclcpp::Node("backup_server_node") {
    initializeParameters();
    initializeROSComponents();
    tf_success_ = false;
    RCLCPP_INFO(this->get_logger(), "Initializing Backup server...");
}

void BackUpServer::initializeParameters() {
    declare_parameter<std::string>("from_frame", "robot_base_link");
    declare_parameter<std::string>("to_frame", "cart");
    declare_parameter<double>("backup_distance", 0.0);
    declare_parameter<double>("tolerance", 0.0);
}

void BackUpServer::initializeROSComponents() {
    vel_pub_ = create_publisher<CmdVel>("/cmd_vel", 10);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    srv_ = create_service<BackUp>("/backup", std::bind(&BackUpServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

Pose BackUpServer::getTransform(const std::string& fromFrame, const std::string& toFrame) {
    geometry_msgs::msg::TransformStamped transform;
    auto pose = geometry_msgs::msg::PoseStamped();

    try {
        transform = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        handleTransformException(fromFrame, toFrame, ex.what());
        return getDefaultPose();
    }

    auto translation = transform.transform.translation;
    auto rotation = transform.transform.rotation;

    pose.pose.position.x = translation.x;
    pose.pose.position.y = translation.y;
    pose.pose.position.z = translation.z;
    pose.pose.orientation.x = rotation.x;
    pose.pose.orientation.y = rotation.y;
    pose.pose.orientation.z = rotation.z;
    pose.pose.orientation.w = rotation.w;

    tf_success_ = true;
    return pose;
}

void BackUpServer::handleTransformException(const std::string& fromFrame, const std::string& toFrame, const std::string& errorMsg) {
    RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", fromFrame.c_str(), toFrame.c_str(), errorMsg.c_str());
    tf_success_ = false;
}

Pose BackUpServer::getDefaultPose() {
    Pose pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "robot_base_link";
    pose.pose.orientation.w = 1.0;
    return pose;
}

void BackUpServer::service_callback(const std::shared_ptr<BackUp::Request> req, const std::shared_ptr<BackUp::Response> res) {
    auto fromFrame = get_parameter("from_frame").as_string();
    auto toFrame = get_parameter("to_frame").as_string();
    auto backupDist = get_parameter("backup_distance").as_double();
    auto TOLERANCE = get_parameter("tolerance").as_double();

    if (!req->is_backup_call) {
        res->is_success = false;
        return;
    }

    RCLCPP_INFO(get_logger(), "Start /backup service. Robot is moving backward");

    const float VEL = 0.05;
    CmdVel vel_msg;
    rclcpp::Rate loop_rate(10);

    Pose pose = getTransform(fromFrame, toFrame);
    float current_x = pose.pose.position.x;
    float current_y = pose.pose.position.y;

    if (!tf_success_) {
        res->is_success = false;
        RCLCPP_INFO(get_logger(), "Cannot find transform from %s to %s", fromFrame.c_str(), toFrame.c_str());
        return;
    }

    if (std::abs(current_y) >= static_cast<float>(TOLERANCE)) {
        res->is_success = false;
        RCLCPP_INFO(get_logger(), "Cannot backup. Offset in Y is more than tolerance %f", TOLERANCE);
        return;
    }

    while (current_x < static_cast<float>(backupDist)) {
        pose = getTransform(fromFrame, toFrame);
        current_x = pose.pose.position.x;
        current_y = pose.pose.position.y;
        vel_msg.linear.x = (-1) * VEL;
        vel_msg.angular.z = 0.0;
        vel_pub_->publish(vel_msg);
        loop_rate.sleep();
    }

    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    vel_pub_->publish(vel_msg);
    res->is_success = true;
}

