#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class ImageProjection(Node):

    def __init__(self):
        super().__init__('image_projection')

        descriptor_top = ParameterDescriptor(
            description='projection range top.',
            integer_range=[IntegerRange(from_value=0, to_value=200, step=1)]
        )
        descriptor_bottom = ParameterDescriptor(
            description='projection range bottom.',
            integer_range=[IntegerRange(from_value=0, to_value=400, step=1)]
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.extrinsic_camera_calibration.top_x', 60, descriptor_top),
                ('camera.extrinsic_camera_calibration.top_y', 40, descriptor_top),
                ('camera.extrinsic_camera_calibration.bottom_x', 180, descriptor_bottom),
                ('camera.extrinsic_camera_calibration.bottom_y', 120, descriptor_bottom),
                ('is_extrinsic_camera_calibration_mode', True)
            ]
        )

        self.top_x = self.get_parameter('camera.extrinsic_camera_calibration.top_x').value
        self.top_y = self.get_parameter('camera.extrinsic_camera_calibration.top_y').value
        self.bottom_x = self.get_parameter('camera.extrinsic_camera_calibration.bottom_x').value
        self.bottom_y = self.get_parameter('camera.extrinsic_camera_calibration.bottom_y').value

        self.is_calibration_mode = self.get_parameter(
            'is_extrinsic_camera_calibration_mode').value

        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetImageProjectionParam)

        self.sub_image_type = 'compressed'
        self.pub_image_type = 'raw'

        # Subscribers
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

        # Publishers
        if self.pub_image_type == 'raw':
            self.pub_image_projected = self.create_publisher(Image, '/camera/image_output', 1)
        else:
            self.pub_image_projected = self.create_publisher(
                CompressedImage, '/camera/image_output/compressed', 1
            )

        if self.is_calibration_mode:
            if self.pub_image_type == 'raw':
                self.pub_image_calib = self.create_publisher(
                    Image, '/camera/image_calib', 1
                )
            else:
                self.pub_image_calib = self.create_publisher(
                    CompressedImage, '/camera/image_calib/compressed', 1
                )

        self.cvBridge = CvBridge()

    def cbGetImageProjectionParam(self, params):
        for param in params:
            if param.name.endswith('top_x'):
                self.top_x = param.value
            if param.name.endswith('top_y'):
                self.top_y = param.value
            if param.name.endswith('bottom_x'):
                self.bottom_x = param.value
            if param.name.endswith('bottom_y'):
                self.bottom_y = param.value

        return SetParametersResult(successful=True)

    def cbImageProjection(self, msg_img):

        # Decode image
        if self.sub_image_type == 'compressed':
            arr = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        else:
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, 'bgr8')

        # Get image resolution dynamically
        h_img, w_img, _ = cv_image_original.shape
        cx = w_img // 2
        cy = h_img // 2

        top_x, top_y = self.top_x, self.top_y
        bottom_x, bottom_y = self.bottom_x, self.bottom_y

        # ------------------ Calibration Visualization ------------------
        if self.is_calibration_mode:
            img_calib = cv_image_original.copy()

            # TOP LINE
            cv2.line(img_calib, (cx - top_x, cy - top_y),
                     (cx + top_x, cy - top_y), (0, 0, 255), 2)

            # BOTTOM LINE
            cv2.line(img_calib, (cx - bottom_x, cy + bottom_y),
                     (cx + bottom_x, cy + bottom_y), (0, 0, 255), 2)

            # LEFT SIDE
            cv2.line(img_calib, (cx - bottom_x, cy + bottom_y),
                     (cx - top_x, cy - top_y), (0, 0, 255), 2)

            # RIGHT SIDE
            cv2.line(img_calib, (cx + bottom_x, cy + bottom_y),
                     (cx + top_x, cy - top_y), (0, 0, 255), 2)

            if self.pub_image_type == 'raw':
                self.pub_image_calib.publish(
                    self.cvBridge.cv2_to_imgmsg(img_calib, 'bgr8'))
            else:
                self.pub_image_calib.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(img_calib, 'jpg'))

        # ------------------ Homography Transform ------------------
        pts_src = np.array([
            [cx - top_x, cy - top_y],
            [cx + top_x, cy - top_y],
            [cx + bottom_x, cy + bottom_y],
            [cx - bottom_x, cy + bottom_y]
        ])

        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        h_matrix, status = cv2.findHomography(pts_src, pts_dst)

        warped = cv2.warpPerspective(cv_image_original, h_matrix, (1000, 600))

        # Publish transformed image
        if self.pub_image_type == 'raw':
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_imgmsg(warped, 'bgr8'))
        else:
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(warped, 'jpg'))

def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()