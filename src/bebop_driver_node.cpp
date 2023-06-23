#include "ros2_bebop_driver/bebop_driver_node.hpp"

#include <chrono>
#include <cstdio>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/image_encodings.hpp>

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
	// Make std cout/cerr unbuffered
	// This can create performance issues
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

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
	    "package://ros2_bebop_driver/config/bebop2_camera_calib.yaml");
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

    // Camera
    publisher_camera =
	image_transport::create_camera_publisher(this, "camera/image_raw");
    bebop->startStreaming();
    if (bebop->isStreamingStarted()) {
	// Camera info publication on a regular basis
	camera_timer = this->create_wall_timer(
	    30ms, std::bind(&BebopDriverNode::publishCamera, this));
	RCLCPP_INFO(this->get_logger(), "Streaming is started");
    } else {
	RCLCPP_ERROR(this->get_logger(), "Failed to start streaming");
    }

    // Odometry: published at 15Hz
    publisher_odometry =
	this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_timer = this->create_wall_timer(
	66ms, std::bind(&BebopDriverNode::publishOdometry, this));
}

void BebopDriverNode::publishCamera(void) {
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(
	new sensor_msgs::msg::CameraInfo(cinfo_manager->getCameraInfo()));

    rclcpp::Time timestamp = this->get_clock()->now();

    camera_info_msg->header.stamp = timestamp;
    camera_info_msg->header.frame_id = camera_frame_id;

    sensor_msgs::msg::Image::SharedPtr img_msg =
	std::make_shared<sensor_msgs::msg::Image>();

    auto header = std::make_shared<std_msgs::msg::Header>();
    img_msg->header.stamp = timestamp;
    img_msg->header.frame_id = camera_frame_id;
    this->bebop->getFrontCameraFrame(img_msg->data, img_msg->width,
				     img_msg->height);
    img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    /* img_msg->is_bigendian = ; TODO*/
    img_msg->step = 3 * img_msg->width;

    publisher_camera.publish(img_msg, camera_info_msg);
}

void BebopDriverNode::publishOdometry(void) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = odom_frame_id;
    message.child_frame_id = "base_link";

    // The position and orientation
    message.pose.pose.position.x = 0.0;
    message.pose.pose.position.y = 0.0;
    message.pose.pose.position.z = 0.0;
    /* message.pose.pose.orientation = {0.0, 0.0, 0.0}; */
    /* message.pose.covariance = ; */

    // The velocities
    message.twist.twist.linear.x = 0.0;
    message.twist.twist.linear.y = 0.0;
    message.twist.twist.linear.z = 0.0;
    /* message.twist.twist.angular = {0.0, 0.0, 0.0}; */
    /* message.twist.covariance = ; */

    publisher_odometry->publish(message);
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
