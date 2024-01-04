#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "shelf_detect/shelf_detect_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShelfDetectionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}