#include "ros2_bebop_driver/bebop_driver_node.hpp"

#include <cstdio>

namespace bebop_driver {
BebopDriverNode::BebopDriverNode()
    : rclcpp::Node("bebop_driver_node"), bebop(std::make_shared<Bebop>()) {}

}  // namespace bebop_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bebop_driver::BebopDriverNode>());
    rclcpp::shutdown();
    return 0;
}
