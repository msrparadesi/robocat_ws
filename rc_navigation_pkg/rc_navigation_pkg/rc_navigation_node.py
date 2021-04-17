#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Modifications Copyright Martin Paradesi. All Rights Reserved.               #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
rc_navigation_node.py

This module decides the action messages (servo control messages specifically angle
and throttle) to be sent out using the mouse detection flag from mouse_detection_node.

The node defines:
    mouse_detection_subscriber: A subscriber to the /mouse_detection_pkg/mouse_detection_topic
                                published by the mouse_detection_pkg with the mouse flag.
    The node defines:
    action_publisher: A publisher to publish the action (angle and throttle values).
"""
import time
import signal
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from deepracer_interfaces_pkg.msg import (MouseDetectionMsg,
                                          ServoCtrlMsg)
from rc_navigation_pkg import (constants,
                                utils)


class RCNavigationNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and throttle) to be sent out using the detection deltas from mouse_detection_node.
    """

    def __init__(self, qos_profile):
        """Create a RCNavigationNode.
        """
        super().__init__('rc_navigation_node')
        self.get_logger().info("rc_navigation_node started.")

        # Double buffer to hold the mouse flag from Mouse Detection.
        self.mouse_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Create subscription to detection mouse from mouse_detection_node.
        self.mouse_detection_subscriber = \
            self.create_subscription(MouseDetectionMsg,
                                     constants.MOUSE_DETECTION_TOPIC,
                                     self.mouse_detection_cb,
                                     qos_profile)

        # Creating publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(ServoCtrlMsg,
                                                      constants.ACTION_PUBLISH_TOPIC,
                                                      qos_profile)

        # Initializing the msg to be published.
        msg = ServoCtrlMsg()
        msg.angle, msg.throttle = constants.ActionValues.DEFAULT, constants.ActionValues.DEFAULT

        self.lock = threading.Lock()
        # Default maximum speed percentage (updated as per request using service call).
        self.max_speed_pct = constants.MAX_SPEED_PCT

        # Create a background servo publish thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.action_publish, args=(msg,))
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input mouse flag: {constants.MOUSE_DETECTION_TOPIC}")

    def wait_for_thread(self):
        """Function which joins the created background thread.
        """
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread.
        """
        self.stop_thread = True

    def set_max_speed_cb(self, req, res):
        """Callback which dynamically sets the max_speed_pct.

        Args:
            req (SetMaxSpeedSrv.Request): Request object with the updated
                                          max speed percentage.
            res (SetMaxSpeedSrv.Response): Response object with error(int) flag
                                           indicating successful max speed pct
                                           update.

        Returns:
            SetMaxSpeedSrv.Response: Response object with error(int) flag indicating
                                             successful max speed pct update.

        """
        self.lock.acquire()
        try:
            self.max_speed_pct = req.max_speed_pct
            self.get_logger().info(f"Incoming request: max_speed_pct: {req.max_speed_pct}")
            res.error = 0
        except Exception as ex:
            self.get_logger().error(f"Failed set max speed pct: {ex}")
            res.error = 1
        finally:
            self.lock.release()
        return res

    def mouse_detection_cb(self, mouse_flag):
        """Call back for whenever mouse flag for a perception
           is received from mouse_detection_node.

        Args:
            mouse_flag (MouseDetectionMsg): Message containing the indicator whether
                                            a mouse was detected or not.
        """

        self.mouse_buffer.put(mouse_flag)

    def action_publish(self, msg):
        """Function which runs in a separate thread to read object detection delta
           from double buffer, decides the action and sends it to servo.

        Args:
            msg: detection_delta (DetectionDeltaMsg): Message containing the normalized
                 detection delta in x and y axes respectively passed as a list.
        """
        try:
            while not self.stop_thread:
                # Get a new message to plan action on
                mouse_flag = self.mouse_buffer.get()

                # Publish msg based on mouse detection flag
                if mouse_flag.is_mouse:
                    # Move forward
                    msg.angle, msg.throttle = constants.ACTION_SPACE[2][constants.ActionSpaceKeys.ANGLE],\
                                                  constants.ACTION_SPACE[2][constants.ActionSpaceKeys.THROTTLE]
                    self.get_logger().info(f"Forward: {msg.angle}, {msg.throttle}")
                    self.action_publisher.publish(msg)
                    time.sleep(constants.DEFAULT_SLEEP)
                    # Move backward
                    msg.angle, msg.throttle = constants.ACTION_SPACE[3][constants.ActionSpaceKeys.ANGLE],\
                                                  constants.ACTION_SPACE[3][constants.ActionSpaceKeys.THROTTLE]
                    self.get_logger().info(f"Reverse: {msg.angle}, {msg.throttle}")
                    self.action_publisher.publish(msg)
                else:
                    self.get_logger().info("No Action")
                    msg.angle, msg.throttle = constants.ACTION_SPACE[1][constants.ActionSpaceKeys.ANGLE],\
                                                  constants.ACTION_SPACE[1][constants.ActionSpaceKeys.THROTTLE]
                    self.action_publisher.publish(msg)
                # Sleep for a default amount of time before checking if new data is available.
                time.sleep(constants.DEFAULT_SLEEP)
                # If new data is not available within default time, do nothing.
                while self.mouse_buffer.is_empty() and not self.stop_thread:
                    msg.angle, msg.throttle = constants.ACTION_SPACE[1][constants.ActionSpaceKeys.ANGLE],\
                                                  constants.ACTION_SPACE[1][constants.ActionSpaceKeys.THROTTLE]
                    # Publish blind action
                    self.action_publisher.publish(msg)
                    # Sleep before checking if new data is available.
                    time.sleep(constants.DEFAULT_SLEEP)
        except Exception as ex:
            self.get_logger().error(f"Failed to publish action to servo: {ex}")
            # Stop the car
            msg.angle, msg.throttle = constants.ActionValues.DEFAULT, constants.ActionValues.DEFAULT
            self.action_publisher.publish(msg)
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        rc_navigation_node = RCNavigationNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            rc_navigation_node.get_logger().info("Signal Handler initiated")
            rc_navigation_node.thread_shutdown()
            rc_navigation_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(rc_navigation_node, executor)
    except Exception as ex:
        rc_navigation_node.get_logger().error(f"Exception in RCNavigationNode: {ex}")
        rc_navigation_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rc_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

