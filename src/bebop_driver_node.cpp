#include "ros2_bebop_driver/bebop_driver_node.hpp"

#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

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

    cinfo_manager =
	std::make_shared<camera_info_manager::CameraInfoManager>(this);
    {
	/* get ROS2 config parameter for camera calibration file */
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The path to the yaml camera calibration file";
	auto camera_calibration_file_param = this->declare_parameter(
	    "camera_calibration_file",
	    "package://ros2_bebop_driver/config/bebop2.yaml");
	cinfo_manager->setCameraName("bebop_front");
	cinfo_manager->loadCameraInfo(camera_calibration_file_param);
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The frame id of the camera";
	camera_frame_id =
	    this->declare_parameter("camera_frame_id", "camera_optical");
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description =
	    "The frame id of the odometry frame of reference";
	odom_frame_id = this->declare_parameter("odom_frame_id", "odom");
    }

    auto bebop_ip = this->get_parameter("bebop_ip").as_string();
    auto bebop_port = this->get_parameter("bebop_port").as_int();
    RCLCPP_INFO(this->get_logger(), "Connecting to the bebop %s:%d",
		bebop_ip.c_str(), bebop_port);
    bebop->connect(bebop_ip, bebop_port);
    RCLCPP_INFO(this->get_logger(), "Connected");

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

    // cmdvel
    subscription_cmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
	"cmd_vel", 1,
	std::bind(&BebopDriverNode::cmdVelCallback, this,
		  std::placeholders::_1));

    // Camera info publication on a regular basis
    camera_timer = this->create_wall_timer(
	30ms, std::bind(&BebopDriverNode::publishCamera, this));
}

void BebopDriverNode::publishCamera(void) {
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(
	new sensor_msgs::msg::CameraInfo(cinfo_manager->getCameraInfo()));

    rclcpp::Time timestamp = this->get_clock()->now();

    camera_info_msg->header.stamp = timestamp;
    camera_info_msg->header.frame_id = camera_frame_id;

    // publisher_camera.publish(image_msg, camera_info_msg);
}

void BebopDriverNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double roll = msg->linear.y;
    double pitch = msg->linear.x;
    double gaz_speed = msg->linear.z;
    double yaw_speed = msg->angular.z;
    bebop->move(roll, pitch, gaz_speed, yaw_speed);
}

}  // namespace bebop_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bebop_driver::BebopDriverNode>());
    rclcpp::shutdown();
    return 0;
}
