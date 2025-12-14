# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry

# import json
# import os
# import math


# class OdomLogger(Node):
#     """
#     Subscribes to /odom and logs x, y, yaw into a JSON file:
#     slam_lane_tracking_pkg/scripts/pose_data.json
#     """

#     def __init__(self):
#         super().__init__('odom_logger')

#         # Default path to JSON file inside this package’s source folder
#         # Adjust username if needed.
#         default_pose_path = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/scripts/pose_data.json'

#         self.declare_parameter('pose_file_path', default_pose_path)
#         self.pose_file_path = self.get_parameter('pose_file_path').get_parameter_value().string_value

#         os.makedirs(os.path.dirname(self.pose_file_path), exist_ok=True)

#         self.X_list = []
#         self.Y_list = []
#         self.yaw_list = []

#         self.subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )

#         self.get_logger().info(f'OdomLogger started, writing to: {self.pose_file_path}')

#     def quaternion_to_yaw(self, x, y, z, w):
#         # Standard yaw from quaternion
#         siny_cosp = 2.0 * (w * z + x * y)
#         cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     def read_existing_data(self):
#         if os.path.exists(self.pose_file_path):
#             try:
#                 with open(self.pose_file_path, 'r') as f:
#                     return json.load(f)
#             except (json.JSONDecodeError, OSError):
#                 self.get_logger().warn('Could not read existing pose JSON. Starting fresh.')
#         return {}

#     def odom_callback(self, msg: Odometry):
#         pos = msg.pose.pose.position
#         ori = msg.pose.pose.orientation

#         yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

#         x_rounded = round(pos.x, 2)
#         y_rounded = round(pos.y, 2)
#         yaw_rounded = round(yaw, 2)

#         self.X_list.append(x_rounded)
#         self.Y_list.append(y_rounded)
#         self.yaw_list.append(yaw_rounded)

#         data = self.read_existing_data()
#         data["Robot Position"] = {
#             "X": self.X_list,
#             "Y": self.Y_list,
#             "Yaw": self.yaw_list
#         }

#         try:
#             with open(self.pose_file_path, 'w') as f:
#                 json.dump(data, f, indent=4)
#             self.get_logger().info(f'Updated pose: x={x_rounded}, y={y_rounded}, yaw={yaw_rounded}')
#         except OSError as e:
#             self.get_logger().error(f'Failed to write pose file: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import json
import os
import math

class TFLogger(Node):
    """
    Logs robot pose (x, y, yaw) into a JSON file using TF2 logic.
    Target File: slam_lane_tracking_pkg/scripts/pose_data.json
    """

    def __init__(self):
        super().__init__('tf_logger')

        # Default path to JSON file inside this package’s source folder
        # Adjust username if needed.
        default_pose_path = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/scripts/pose_data.json'

        self.declare_parameter('pose_file_path', default_pose_path)
        self.pose_file_path = self.get_parameter('pose_file_path').get_parameter_value().string_value

        os.makedirs(os.path.dirname(self.pose_file_path), exist_ok=True)

        self.X_list = []
        self.Y_list = []
        self.yaw_list = []

        self.get_logger().info(f'TFLogger started, writing to: {self.pose_file_path}')

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a timer to look up the transform every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def quaternion_to_yaw(self, x, y, z, w):
        # Standard yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def read_existing_data(self):
        if os.path.exists(self.pose_file_path):
            try:
                with open(self.pose_file_path, 'r') as f:
                    return json.load(f)
            except (json.JSONDecodeError, OSError):
                self.get_logger().warn('Could not read existing pose JSON. Starting fresh.')
        return {}

    def timer_callback(self):
        # Try to look up the transform from 'map' to 'base_link'
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time()
            )

            # Extract data from the transform
            pos_x = trans.transform.translation.x
            pos_y = trans.transform.translation.y
            
            ori = trans.transform.rotation
            yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

            # Round data
            x_rounded = round(pos_x, 2)
            y_rounded = round(pos_y, 2)
            yaw_rounded = round(yaw, 2)

            # Append to lists
            self.X_list.append(x_rounded)
            self.Y_list.append(y_rounded)
            self.yaw_list.append(yaw_rounded)

            # Save to JSON
            data = self.read_existing_data()
            data["Robot Position"] = {
                "X": self.X_list,
                "Y": self.Y_list,
                "Yaw": self.yaw_list
            }

            try:
                with open(self.pose_file_path, 'w') as f:
                    json.dump(data, f, indent=4)
                # Optional: Uncomment below to see logs in terminal
                # self.get_logger().info(f'TF Pose: x={x_rounded}, y={y_rounded}, yaw={yaw_rounded}')
            except OSError as e:
                self.get_logger().error(f'Failed to write pose file: {e}')

        except (LookupException, ConnectivityException, ExtrapolationException):
            # These exceptions happen when TF is not yet ready or frames are missing
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()