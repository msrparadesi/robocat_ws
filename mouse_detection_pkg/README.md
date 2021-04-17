# AWS DeepRacer Mouse Detection Package

## Overview

The Mouse Detection ROS package creates the mouse_detection_node which is responsible for collecting sensor data (camera images) from sensor_fusion_pkg and running them through the object detection model to find a mouse and providing a boolean value whether a mouse was detected or not. This boolean value is published using a ROS publisher as MouseDetectionMsg data. For more information about the RoboCat sample project, see [RoboCat sample project](https://github.com/msrparadesi/robocat_ws).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the RoboCat sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The object_detection_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support RoboCat sample project.


## Downloading and Building

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

1. Build the mouse_detection_pkg and deepracer_interfaces_pkg:

        cd ~/robocat_ws/ && colcon build --packages-select object_detection_pkg deepracer_interfaces_pkg


## Usage

Although the **mouse_detection_node** is built to work with the RoboCat sample project, it can be run independently for development/testing/debugging purposes.

### Run the node

Configure the launch file to specify which device to use for inference (for more details, see the extended configuration section below). To launch the built mouse_detection_node as root user on the AWS DeepRacer device, open up another terminal on the device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the RoboCat workspace:

        cd ~/robocat_ws/

1. Source the ROS2 Foxy setup bash and OpenVINO bash script:

        source /opt/ros/foxy/setup.bash 
        source /opt/intel/openvino_2021/bin/setupvars.sh 

1. Source the setup script for the installed packages:

        source ~/robocat_ws/install/setup.bash 

1. Launch the mouse_detection_node using the launch script:

        ros2 launch mouse_detection_pkg mouse_detection_pkg_launch.py

## Launch Files

A launch file called mouse_detection_pkg_launch.py is included in this package that gives an example of how to launch the mouse_detection_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='mouse_detection_pkg',
                    namespace='mouse_detection_pkg',
                    executable='mouse_detection_node',
                    name='mouse_detection_node',
                    parameters=[{
                        'DEVICE': 'CPU'
                    }]
                )
            ])

### Configuration File and Parameters

#### mouse_detection_node

| Parameter Name   | Description  |
| ---------------- |  ----------- |
| DEVICE (optional) | Uses CPU for inference by default, even if removed. |


## Node Details

### mouse_detection_node

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| /sensor_fusion_pkg/sensor_msg | EvoSensorMsg | This message holds a list of sensor_msgs/Image objects that are independently collected from different camera sensors. |


#### Published Topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
| mouse_detection_delta | MouseDetectionMsg | Message with boolean value whether a mouse was detected or not. |

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Follow the Leader(FTL) sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
* Instructions to download and optimize the object detection model: [https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/download-and-convert-object-detection-model.md)
