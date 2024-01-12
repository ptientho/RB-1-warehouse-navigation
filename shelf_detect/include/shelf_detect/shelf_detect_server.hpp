

#ifndef SHELF_DETECT__SHELF_DETECT_SERVER_HPP_
#define SHELF_DETECT__SHELF_DETECT_SERVER_HPP_

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "shelf_detect_msg/srv/go_to_shelf.hpp"
#include <cstddef>
#include <memory>
#include <vector>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;;
class ShelfDetectionServer : public rclcpp::Node
{
    public:
    using GoToShelf = shelf_detect_msg::srv::GoToShelf;
    using LaserScan = sensor_msgs::msg::LaserScan;
    using Odom = nav_msgs::msg::Odometry;

    ShelfDetectionServer();

    private:
    rclcpp::Service<GoToShelf>::SharedPtr srv_;
    rclcpp::Subscription<LaserScan>::SharedPtr laserSub_;
    rclcpp::Subscription<Odom>::SharedPtr odomSub_;
    rclcpp::CallbackGroup::SharedPtr subGrp_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    // timer use for dynamic tf publishing
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    
    // tf objects
    // static tf publishes frame once and for all, making it static
    //std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
    // dynamic tf publishes frame every iteration, measuring its pose to parent frame
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_dyn_;
    
    //odom data
    double robot_yaw;
    Odom::SharedPtr odom_data = nullptr;
    LaserScan::SharedPtr laser_data = nullptr;

    //leg properties
    float leg1_range = NULL;
    float leg2_range = NULL;
    float angle_diff = NULL;
    int leg1_idx_start;
    int leg2_idx_start;
    int leg1_idx_end;
    int leg2_idx_end;
    int leg1_idx_mid;
    int leg2_idx_mid;
    bool shelf_found;
    
    void service_callback(const std::shared_ptr<GoToShelf::Request> req, const std::shared_ptr<GoToShelf::Response> rsp);
    void odom_callback(const std::shared_ptr<Odom> msg);
    void laser_callback(const std::shared_ptr<LaserScan> msg);
    
    /* method to design algorithm for shelf detection whether it's available or not */
    void detect_shelf();
    
    /* method to design algorithm for front shelf range computation */
    std::vector<float> compute_front_shelf_distance();
    
    /* broadcast shelf frame wrt robot_base_link */
    void publish_shelf_frame();

 
};



#endif // SHELF_DETECT__SHELF_DETECT_SERVER_HPP_