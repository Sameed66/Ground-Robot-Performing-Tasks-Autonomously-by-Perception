#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import json
import os


class CentroidLogger(Node):
    """
    Subscribes to a camera image, detects lane lines, computes centroids,
    and logs them to centroid_data.json for use by virtual_obstacles_node.

    JSON format:
    {
      "Lanes Data": {
        "Left X":  [...],
        "Right X": [...]
      },
      "Centre Data": {
        "Centre X": [...]
      }
    }
    """

    def __init__(self):
        super().__init__('centroid_logger')

        # Parameters
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter(
            'centroid_file_path',
            '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/scripts/centroid_data.json'
        )

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.centroid_file_path = self.get_parameter('centroid_file_path').get_parameter_value().string_value

        # Ensure directory exists
        os.makedirs(os.path.dirname(self.centroid_file_path), exist_ok=True)

        self.bridge = CvBridge()

        # Lists to store values
        self.left_x_list = []
        self.right_x_list = []
        self.centre_x_list = []

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'CentroidLogger started. Subscribing to {self.image_topic}')
        self.get_logger().info(f'Logging centroids to: {self.centroid_file_path}')

    def read_existing_data(self):
        if os.path.exists(self.centroid_file_path):
            try:
                with open(self.centroid_file_path, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError:
                self.get_logger().warn('Existing centroid_data.json is invalid. Starting fresh.')
                return {}
        return {}

    def save_centroids(self, left_x, right_x, centre_x):
        # Append new samples
        self.left_x_list.append(left_x)
        self.right_x_list.append(right_x)
        self.centre_x_list.append(centre_x)

        data = self.read_existing_data()

        data["Lanes Data"] = {
            "Left X": self.left_x_list,
            "Right X": self.right_x_list
        }
        data["Centre Data"] = {
            "Centre X": self.centre_x_list
        }

        try:
            with open(self.centroid_file_path, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(
                f'Saved centroids: Left X={left_x}, Right X={right_x}, Centre X={centre_x}'
            )
        except OSError as e:
            self.get_logger().error(f'Failed to write centroid_data.json: {e}')

    def image_callback(self, image_msg: Image):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Convert to HSV
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # White mask
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv_image, lower_white, upper_white)

        # Yellow mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Combine masks
        final_mask = cv2.bitwise_or(mask_white, mask_yellow)

        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        num_contours = len(contours)
        self.get_logger().debug(f'Total number of detected contours: {num_contours}')

        if not contours:
            self.get_logger().info('No contours found, skipping frame.')
            return

        # Take second and third largest contours (same as your ROS1 code)
        largest_contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:3]

        line_right = {}
        line_left = {}

        highlighted_lane_lines = cv2.bitwise_and(rgb_image, rgb_image, mask=final_mask)

        if len(largest_contours) > 0:
            M_right = cv2.moments(largest_contours[0])
            if M_right["m00"] != 0:
                line_right['x'] = int(M_right["m10"] / M_right["m00"])
                line_right['y'] = int(M_right["m01"] / M_right["m00"])
            else:
                self.get_logger().warn('Right contour moment m00 is zero, skipping.')
                return

        if len(largest_contours) > 1:
            M_left = cv2.moments(largest_contours[1])
            if M_left["m00"] != 0:
                line_left['x'] = int(M_left["m10"] / M_left["m00"])
                line_left['y'] = int(M_left["m01"] / M_left["m00"])
            else:
                self.get_logger().warn('Left contour moment m00 is zero, skipping.')
                return

        if 'x' not in line_left or 'x' not in line_right:
            self.get_logger().info('Did not detect both lane lines, skipping.')
            return

        # Image width for camera center
        height, width, _ = highlighted_lane_lines.shape
        camera_center = width / 2.0

        left_x = line_left['x']
        right_x = line_right['x']
        centre_x = camera_center  # treat image center as robot centre

        self.save_centroids(left_x, right_x, centre_x)


def main(args=None):
    rclpy.init(args=args)
    node = CentroidLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()