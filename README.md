# ROS2 bebop driver

This is a ROS2 rewrite derived from the original [ROS1 bebop_autonomy package](https://github.com/AutonomyLab/bebop_autonomy).

If you wish, you could be using [ros1 bridge](https://github.com/ros2/ros1_bridge) to use the original bebop_autonomy ROS1 package. 

This is clearly alpha release with some basic functionnalities which will hopefully grow from time to time.

This work is based on the [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) work of Mani Monajjemi and the work on [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport) by Clyde McQueen.

## Using it

You need [ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk) in your workspace as well as this package, build it and run.

```
cd ros2_ws
git clone https://github.com/jeremyfix/ros2_bebop_driver.git src/ros2_bebop_driver
git clone https://github.com/jeremyfix/ros2_parrot_arsdk.git src/ros2_parrot_arsdk
colcon build --symlink-install
```

And then you should be able to run the launch file

```
source ros2_ws/install/setup.bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=YOUR.BEBOP.IP.ADDRESS 
```

## Features and roadmap

| Feature | Status | Notes |
| --- | --- | --- |
| SDK Version | 3.14.0 | Depends on [ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk) |
| Support for Parrot Bebop 2 | Yes | |
| Support for Parrot Bebop 1 | No | Should not be too difficult to implement but I cannot test | 
| Core piloting (takeoff, land, move, ..) | Yes | |
| H264 video decoding | Yes | |
| ROS Camera interface | Yes | |
| Publish bebop states as ROS topics | No | | 
| TF publisher | Yes | |
| Odometry publisher | Yes | |

The odometry has been not been extensively tested. It is a direct port of the code from [ROS1 bebop_autonomy package](https://github.com/AutonomyLab/bebop_autonomy). However, when I tested it, it seemed to me there was an accumulated error during motion. Not a drift but an offset.

## Debugging

If you need to debug the execution of the node, you can :

```
ros2 run --prefix 'gdb -ex run --args' ros2_bebop_driver bebop_driver --ros-args -p bebop_ip:=192.168.50.62
```

you may need to customize the drone IP.
