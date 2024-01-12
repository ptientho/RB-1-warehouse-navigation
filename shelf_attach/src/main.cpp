#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "shelf_attach/shelf_attach_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AttachShelfServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}