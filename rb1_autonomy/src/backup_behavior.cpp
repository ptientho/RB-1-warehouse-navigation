#include "rb1_autonomy/backup_behavior.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <ratio>

using namespace std::chrono_literals;
NodeStatus BackUpActionNode::tick(){

    // publish cmd_vel to back up for 5 second and then stop
    float vel = 0.05;
    CmdVel vel_msg;
    vel_msg.linear.x = (-1) * vel;

    for (int i = 0; i <= 10; i++){
    
        vel_pub_->publish(vel_msg);
        std::this_thread::sleep_for(0.5s); //0.5s
    }

    vel_msg.linear.x = 0.0;
    vel_pub_->publish(vel_msg);

    return NodeStatus::SUCCESS;
}