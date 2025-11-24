#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, [AuTURBO] Ki Hoon Kim (https://github.com/auturbo), Gilbert
# Updated: extrinsic calibration made resolution-independent and parameter-driven.

import cv2
from cv_bridge import CvBridge
import numpy as np

from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image


class ImageProjection(Node):

    def __init__(self):
        super().__init__('image_projection')

        # ---------------------- parameter descriptors -----------------------
        parameter_descriptor_top = ParameterDescriptor(
            description='projection range top.',
            integer_range=[IntegerRange(from_value=0, to_value=120, step=1)]
        )
        parameter_descriptor_bottom = ParameterDescriptor(
            description='projection range bottom.',
            integer_range=[IntegerRange(from_value=0, to_value=320, step=1)]
        )

        # --------------------------- parameters ------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.extrinsic_camera_calibration.top_x',
                 80, parameter_descriptor_top),
                ('camera.extrinsic_camera_calibration.top_y',
                 40, parameter_descriptor_top),
                ('camera.extrinsic_camera_calibration.bottom_x',
                 160, parameter_descriptor_bottom),
                ('camera.extrinsic_camera_calibration.bottom_y',
                 80, parameter_descriptor_bottom),
                ('is_extrinsic_camera_calibration_mode', False)
            ]
        )

        self.top_x = self.get_parameter(
            'camera.extrinsic_camera_calibration.top_x'
        ).get_parameter_value().integer_value
        self.top_y = self.get_parameter(
            'camera.extrinsic_camera_calibration.top_y'
        ).get_parameter_value().integer_value
        self.bottom_x = self.get_parameter(
            'camera.extrinsic_camera_calibration.bottom_x'
        ).get_parameter_value().integer_value
        self.bottom_y = self.get_parameter(
            'camera.extrinsic_camera_calibration.bottom_y'
        ).get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_extrinsic_camera_calibration_mode'
        ).get_parameter_value().bool_value

        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetImageProjectionParam)

        # ---------------------- topic configuration -------------------------
        # you can choose 'compressed' or 'raw'
        self.sub_image_type = 'compressed'
        self.pub_image_type = 'raw'

        # subscriber
        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/camera/image_input/compressed',
                self.cbImageProjection,
                1
            )
        else:
            self.sub_image_original = self.create_subscription(
                Image,
                '/camera/image_input',
                self.cbImageProjection,
                1
            )

        # publishers
        if self.pub_image_type == 'compressed':
            self.pub_image_projected = self.create_publisher(
                CompressedImage,
                '/camera/image_output/compressed',
                1
            )
        else:
            self.pub_image_projected = self.create_publisher(
                Image,
                '/camera/image_output',
                1
            )

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_calib = self.create_publisher(
                    CompressedImage,
                    '/camera/image_calib/compressed',
                    1
                )
            else:
                self.pub_image_calib = self.create_publisher(
                    Image,
                    '/camera/image_calib',
                    1
                )

        self.cvBridge = CvBridge()

    # ------------------------------------------------------------------ params
    def cbGetImageProjectionParam(self, parameters):
        """Dynamic reconfigure style callback for ROI parameters."""
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')

            if param.name == 'camera.extrinsic_camera_calibration.top_x':
                self.top_x = int(param.value)
            elif param.name == 'camera.extrinsic_camera_calibration.top_y':
                self.top_y = int(param.value)
            elif param.name == 'camera.extrinsic_camera_calibration.bottom_x':
                self.bottom_x = int(param.value)
            elif param.name == 'camera.extrinsic_camera_calibration.bottom_y':
                self.bottom_y = int(param.value)
            elif param.name == 'is_extrinsic_camera_calibration_mode':
                self.is_calibration_mode = bool(param.value)

        self.get_logger().info(
            f'ROI updated: top({self.top_x}, {self.top_y}), '
            f'bottom({self.bottom_x}, {self.bottom_y})'
        )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- main logic
    def cbImageProjection(self, msg_img):
        # -------------------------- decode image ----------------------------
        if self.sub_image_type == 'compressed':
            np_image_original = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        else:
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, 'bgr8')

        # image geometry
        h, w = cv_image_original.shape[:2]
        cx = w // 2                      # center x

        # parameters from rqt
        top_x = int(self.top_x)
        top_y = int(self.top_y)
        bottom_x = int(self.bottom_x)
        bottom_y = int(self.bottom_y)

        # base vertical positions (fractions of image height)
        base_top_y = int(0.60 * h)       # where the far road is
        base_bottom_y = int(0.95 * h)    # near the bottom of image

        # sliders move lines up (bigger value = higher)
        y_top = base_top_y - top_y
        y_bottom = base_bottom_y - bottom_y

        # clamp within image
        y_top = max(0, min(h - 1, y_top))
        y_bottom = max(0, min(h - 1, y_bottom))

        # ---------------- calibration view: red trapezoid -------------------
        if self.is_calibration_mode:
            cv_image_calib = np.copy(cv_image_original)

            pts_trap = np.array([
                (cx - top_x,    y_top),
                (cx + top_x,    y_top),
                (cx + bottom_x, y_bottom),
                (cx - bottom_x, y_bottom)
            ], dtype=np.int32)

            cv2.polylines(cv_image_calib, [pts_trap], True, (0, 0, 255), 2)

            if self.pub_image_type == 'compressed':
                self.pub_image_calib.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, 'jpg'))
            else:
                self.pub_image_calib.publish(
                    self.cvBridge.cv2_to_imgmsg(cv_image_calib, 'bgr8'))

        # --------------------- prepare for homography -----------------------
        cv_image_blur = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        # 4 source points (trapezoid in original image)
        pts_src = np.array([
            [cx - top_x,    y_top],
            [cx + top_x,    y_top],
            [cx + bottom_x, y_bottom],
            [cx - bottom_x, y_bottom]
        ], dtype=np.float32)

        # 4 destination points (rectangle in bird’s-eye view)
        out_w, out_h = 1000, 600
        pts_dst = np.array([
            [200,        0],
            [800,        0],
            [800,  out_h - 1],
            [200,  out_h - 1]
        ], dtype=np.float32)

        # homography matrix
        H, status = cv2.findHomography(pts_src, pts_dst)
        if H is None:
            self.get_logger().warn('Homography could not be computed.')
            return

        cv_image_homography = cv2.warpPerspective(cv_image_blur, H, (out_w, out_h))

        # fill empty bottom corners with black triangles (same as original)
        triangle1 = np.array([[0, out_h - 1], [0, 340], [200, out_h - 1]], np.int32)
        triangle2 = np.array([[out_w - 1, out_h - 1],
                              [out_w - 1, 340],
                              [800,      out_h - 1]], np.int32)
        cv_image_homography = cv2.fillPoly(cv_image_homography,
                                           [triangle1, triangle2],
                                           (0, 0, 0))

        # ----------------------------- publish ------------------------------
        if self.pub_image_type == 'compressed':
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, 'jpg')
            )
        else:
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_imgmsg(cv_image_homography, 'bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
