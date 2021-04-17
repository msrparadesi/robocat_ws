# AWS DeepRacer RoboCat Navigation Package

## Overview

The RoboCat Navigation ROS package creates the rc_navigation_node which decides the action / controller message to send out using the normalized detection error (delta) received from object_detection_node. For more information about the RoboCat sample project, see [RoboCat sample project](https://github.com/msrparadesi/robocat_ws).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the RoboCat sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The rc_navigation_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support RoboCat sample project.

### Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Clone the entire RoboCat sample project on the DeepRacer device.

        git clone https://github.com/msrparadesi/robocat_ws.git
        cd ~/robocat_ws/

1. Fetch unreleased dependencies:

        cd ~/robocat_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/robocat_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the rc_navigation_pkg and deepracer_interfaces_pkg:

        cd ~/robocat_ws/ && colcon build --packages-select rc_navigation_pkg deepracer_interfaces_pkg


## Usage

Although the **rc_navigation_node** is built to work with the RoboCat sample project, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built rc_navigation_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the RoboCat workspace:

        cd ~/robocat_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/robocat_ws/install/setup.bash 

1. Launch the rc_navigation_node using the launch script:

        ros2 launch rc_navigation_pkg rc_navigation_pkg_launch.py

## Launch Files

A launch file called rc_navigation_pkg_launch.py is included in this package that gives an example of how to launch the rc_navigation_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='rc_navigation_pkg',
                    namespace='rc_navigation_pkg',
                    executable='rc_navigation_node',
                    name='rc_navigation_node'
                )
            ])


## Node Details

### rc_navigation

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
|/mouse_detection_pkg/mouse_detection_topic|MouseDetectionMsg|Message with boolean value whether a mouse was detected or not.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|rc_drive|ServoCtrlMsg|This message is used to send motor throttle and servo steering angle ratios with respect to the device calibration. It can also be used to send raw PWM values for angle and throttle.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|set_max_speed|SetMaxSpeedSrv|Sets Max Speed Percentage Scale for RoboCat Application.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Follow the Leader(FTL) sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
