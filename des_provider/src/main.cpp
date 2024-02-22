#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <functional>
#include <memory>
//#include <mutex>

using namespace std::chrono_literals;
class DesProvider : public rclcpp::Node {

public:
  DesProvider() : rclcpp::Node("goal_destination_provider_node") {

    this->des_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/des_provider", 5);
    this->point_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 5,
            std::bind(&DesProvider::clicked_point_callback, this,
                      std::placeholders::_1));

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&DesProvider::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Starting destination publisher");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr des_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  geometry_msgs::msg::PointStamped clicked_point;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::mutex point_mutex;

  void clicked_point_callback(
      const std::shared_ptr<geometry_msgs::msg::PointStamped> msg) {

    // std::lock_guard<std::mutex> guard(point_mutex);
    if (msg == nullptr) {

      return;
    }
    /*
    if (msg->point.x == 0.0 && msg->point.y == 0.0) {

      return;
    } else {
      clicked_point.header = msg->header;
      clicked_point.point = msg->point;
    }
    */
    clicked_point.header = msg->header;
    clicked_point.point = msg->point;
  }

  void timer_callback() {

    // std::lock_guard<std::mutex> guard(point_mutex);
    this->des_pub_->publish(clicked_point);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesProvider>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}