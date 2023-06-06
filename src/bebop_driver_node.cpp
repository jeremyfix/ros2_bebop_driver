#include "ros2_bebop_driver/bebop_driver_node.hpp"

#include <cstdio>

namespace bebop_driver {

#define SIMPLECALLBACK(subscriber, msgtype, topic, method)              \
    subscriber = this->create_subscription<msgtype>(                    \
	topic, 1,                                                       \
	[this]([[maybe_unused]] const msgtype::SharedPtr msg) -> void { \
	    this->bebop->method();                                      \
	});

BebopDriverNode::BebopDriverNode()
    : rclcpp::Node("bebop_driver_node"), bebop(std::make_shared<Bebop>()) {
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The IP of the bebop to connect to";

	this->declare_parameter("bebop_ip", "192.168.42.1", param_desc);
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The port of the bebop to connect to";

	this->declare_parameter("bebop_port", 44444, param_desc);
    }

    auto bebop_ip = this->get_parameter("bebop_ip").as_string();
    auto bebop_port = this->get_parameter("bebop_port").as_int();
    RCLCPP_INFO(this->get_logger(), "Connecting to the bebop %s:%d",
		bebop_ip.c_str(), bebop_port);
    bebop->connect(bebop_ip, bebop_port);

    // The core functions subscribers
    // For: TakeOff, Land, Emergency, flatTrim, navigateHome, animationFlip,
    // move and moveCamera
    // TakeOff
    SIMPLECALLBACK(subscription_takeoff, std_msgs::msg::Empty, "takeoff",
		   takeOff);
    // Landing
    SIMPLECALLBACK(subscription_land, std_msgs::msg::Empty, "land", land);

    // Emergency
    SIMPLECALLBACK(subscription_emergency, std_msgs::msg::Empty, "reset",
		   emergency);

    // FlatTrim
    SIMPLECALLBACK(subscription_flattrim, std_msgs::msg::Empty, "flattrim",
		   flatTrim);

    // navigateHome
    subscription_navigateHome = this->create_subscription<std_msgs::msg::Bool>(
	"autoflight/navigate_home", 1,
	[this]([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
	    -> void { this->bebop->navigateHome(msg->data); });

    // animationFlip
    subscription_animationFlip =
	this->create_subscription<std_msgs::msg::UInt8>(
	    "flip", 1,
	    [this](const std_msgs::msg::UInt8::SharedPtr msg) -> void {
		this->bebop->animationFlip(msg->data);
	    });
}

}  // namespace bebop_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bebop_driver::BebopDriverNode>());
    rclcpp::shutdown();
    return 0;
}
