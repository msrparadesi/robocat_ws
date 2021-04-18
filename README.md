# AWS DeepRacer RoboCat sample project

## Overview

The RoboCat project is a sample application built on top of the existing AWS DeepRacer application that was conceived as a solution to scare away mice. Mice are noctural animals and can detect movement from far away. The RoboCat project uses an infrared camera attached to an AWS DeepRacer to detect motion in a dark environment and move forward and backward quickly once motion is detected. Motion is detected by using OpenCV library across multiple frames of the video using logic similar to this [PyImageSearch example](https://www.pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/). Most of the code used in the RoboCat project has been derived from the [Follow the Leader(FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project). A short demo of AWS DeepRacer RoboCat sample project is shown in the image below.

![RoboCat Demo](https://github.com/msrparadesi/robocat_ws/blob/main/resources/robocat-demo.gif)

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the RoboCat sample project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page. The RoboCat sample project requires the AWS DeepRacer application to be installed on the device as it leverages most of the packages from the core application.

The following are the additional software and hardware requirements to get the RoboCat sample project to work on the AWS DeepRacer device. 

1. **Obtain an infrared camera:** The infrared camera used in this project was purchased from [Amazon](https://www.amazon.com/gp/product/B07DWWSWNH/). Replace the existing camera(s) on the AWS DeepRacer device with this infrared camera.

1. **Calibrate the AWS DeepRacer (optional):** Follow the [instructions](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) to calibrate the mechanics of your AWS DeepRacer Vehicle. This should be done so that the vehicle performance is optimal and it behaves as expected.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Stop the deepracer-core.service that is currently running on the device:

        systemctl stop deepracer-core

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Clone the entire RoboCat sample project on the DeepRacer device.

        git clone https://github.com/msrparadesi/robocat_ws.git
        cd ~/robocat_ws/

1. Clone the async_web_server_cpp, web_video_server and rplidar_ros dependency packages on the DeepRacer device:

        cd ~/robocat_ws/ && ./install_dependencies.sh

1. Fetch unreleased dependencies:

        cd ~/robocat_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/robocat_ws/ && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the packages in the workspace

        cd ~/robocat_ws/ && colcon build

## Usage

### Run the node

To launch the RoboCat sample application as root user on the AWS DeepRacer device open up another terminal on the device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/robocat_ws/install/setup.bash

1. Launch the nodes required for RoboCat sample project:

        ros2 launch rc_launcher rc_launcher.py

### Enabling “robocat” mode using CLI:

Once the rc_launcher has been kicked-off, open up a adjacent new terminal as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the RoboCat workspace:

        cd ~/robocat_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/robocat_ws/install/setup.bash

1. Set the mode of the DeepRacer via ctrl_pkg to “robocat” using the below ros2 service call:

        ros2 service call /ctrl_pkg/vehicle_state deepracer_interfaces_pkg/srv/ActiveStateSrv "{state: 3}"

1. Enable “robocat” mode using the below ros2 service call

        ros2 service call /ctrl_pkg/enable_state deepracer_interfaces_pkg/srv/EnableStateSrv "{is_active: True}"

## Launch Files

The rc_launcher.py included in this package is the main launcher file that launches all the required nodes for the RoboCat sample project. This launcher file also includes the nodes from the AWS DeepRacer core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            ld = LaunchDescription()
            mouse_detection_node = Node(
                package='mouse_detection_pkg',
                namespace='mouse_detection_pkg',
                executable='mouse_detection_node',
                name='mouse_detection_node',
                parameters=[{
                    'DEVICE': 'CPU'
                }]
                )
            rc_navigation_node = Node(
                package='rc_navigation_pkg',
                namespace='rc_navigation_pkg',
                executable='rc_navigation_node',
                name='rc_navigation_node'
                )
            camera_node = Node(
                package='camera_pkg',
                namespace='camera_pkg',
                executable='camera_node',
                name='camera_node'
            )
            ctrl_node = Node(
                package='ctrl_pkg',
                namespace='ctrl_pkg',
                executable='ctrl_node',
                name='ctrl_node'
            )
            deepracer_navigation_node = Node(
                package='deepracer_navigation_pkg',
                namespace='deepracer_navigation_pkg',
                executable='deepracer_navigation_node',
                name='deepracer_navigation_node'
            )
            software_update_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='software_update_node',
                name='software_update_node'
            )
            model_loader_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='model_loader_node',
                name='model_loader_node'
            )
            otg_control_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='otg_control_node',
                name='otg_control_node'
            )
            network_monitor_node = Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='network_monitor_node',
                name='network_monitor_node'
            )
            device_info_node = Node(
                package='device_info_pkg',
                namespace='device_info_pkg',
                executable='device_info_node',
                name='device_info_node'
            )
            battery_node = Node(
                package='i2c_pkg',
                namespace='i2c_pkg',
                executable='battery_node',
                name='battery_node'
            )
            inference_node = Node(
                package='inference_pkg',
                namespace='inference_pkg',
                executable='inference_node',
                name='inference_node'
            )
            model_optimizer_node = Node(
                package='model_optimizer_pkg',
                namespace='model_optimizer_pkg',
                executable='model_optimizer_node',
                name='model_optimizer_node'
            )
            rplidar_node = Node(
                package='rplidar_ros',
                namespace='rplidar_ros',
                executable='rplidarNode',
                name='rplidarNode',
                parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': True,
                    }]
            )
            sensor_fusion_node = Node(
                package='sensor_fusion_pkg',
                namespace='sensor_fusion_pkg',
                executable='sensor_fusion_node',
                name='sensor_fusion_node'
            )
            servo_node = Node(
                package='servo_pkg',
                namespace='servo_pkg',
                executable='servo_node',
                name='servo_node'
            )
            status_led_node = Node(
                package='status_led_pkg',
                namespace='status_led_pkg',
                executable='status_led_node',
                name='status_led_node'
            )
            usb_monitor_node = Node(
                package='usb_monitor_pkg',
                namespace='usb_monitor_pkg',
                executable='usb_monitor_node',
                name='usb_monitor_node'
            )
            webserver_publisher_node = Node(
                package='webserver_pkg',
                namespace='webserver_pkg',
                executable='webserver_publisher_node',
                name='webserver_publisher_node'
            )
            web_video_server_node = Node(
                package='web_video_server',
                namespace='web_video_server',
                executable='web_video_server',
                name='web_video_server'
            )
            ld.add_action(mouse_detection_node)
            ld.add_action(rc_navigation_node)
            ld.add_action(camera_node)
            ld.add_action(ctrl_node)
            ld.add_action(deepracer_navigation_node)
            ld.add_action(software_update_node)
            ld.add_action(model_loader_node)
            ld.add_action(otg_control_node)
            ld.add_action(network_monitor_node)
            ld.add_action(device_info_node)
            ld.add_action(battery_node)
            ld.add_action(inference_node)
            ld.add_action(model_optimizer_node)
            ld.add_action(rplidar_node)
            ld.add_action(sensor_fusion_node)
            ld.add_action(servo_node)
            ld.add_action(status_led_node)
            ld.add_action(usb_monitor_node)
            ld.add_action(webserver_publisher_node)
            ld.add_action(web_video_server_node)
            return ld

### Configuration File and Parameters

Applies to the object_detection_node

| Parameter Name   | Description  |
| ---------------- |  ----------- |
| DEVICE (optional) | Uses CPU for inference by default, even if removed. |

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Follow the Leader(FTL sample project: [https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project)
* Follow the Leader(FTL) sample project getting started: [https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md)
* Instructions to calibrate your AWS DeepRacer: [https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html)
