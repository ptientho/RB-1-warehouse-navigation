#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class DesProvider : public rclcpp::Node {
public:
  DesProvider() : Node("goal_destination_provider_node") {
    des_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/des_provider", 5);
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 5,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          if (msg != nullptr) {
            clicked_point = *msg;
          }
        });
    timer_ = create_wall_timer(100ms, [this]() { publishDestination(); });
    RCLCPP_INFO(get_logger(), "Starting destination publisher");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr des_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  geometry_msgs::msg::PointStamped clicked_point;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishDestination() {
    des_pub_->publish(clicked_point);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesProvider>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}