# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from PIL import Image
# import numpy as np
# import math
# import json
# import os
# import yaml


# def real_world_to_map(real_x, real_y, map_origin, map_resolution, map_height):
#     """
#     Convert real-world coordinates (meters) to map pixel coordinates.
#     map_origin: [origin_x, origin_y] of bottom-left corner of the map in world frame.
#     """
#     pixel_x = int((real_x - map_origin[0]) / map_resolution)
#     pixel_y = int(map_height - (real_y - map_origin[1]) / map_resolution)
#     return pixel_x, pixel_y


# def load_map_and_metadata(map_path, logger=None):
#     """
#     Load the map image (PGM) and its metadata from the corresponding YAML file.
#     Returns:
#         map_pixels (np.ndarray),
#         map_width (int),
#         map_height (int),
#         map_resolution (float),
#         map_origin (list [x, y])
#     """
#     yaml_path = map_path.replace('.pgm', '.yaml')
#     if not os.path.exists(yaml_path):
#         msg = f"Map YAML file not found: {yaml_path}"
#         if logger:
#             logger.error(msg)
#         else:
#             print(msg)
#         raise FileNotFoundError(msg)

#     with open(yaml_path, 'r') as f:
#         map_info = yaml.safe_load(f)

#     resolution = map_info['resolution']
#     origin_xy = map_info['origin'][:2]  # [x, y]

#     if logger:
#         logger.info(f"Loaded map metadata from {yaml_path}")
#         logger.info(f"Resolution: {resolution}, Origin: {origin_xy}")

#     # Load map image as grayscale
#     if not os.path.exists(map_path):
#         msg = f"Map PGM file not found: {map_path}"
#         if logger:
#             logger.error(msg)
#         else:
#             print(msg)
#         raise FileNotFoundError(msg)

#     img = Image.open(map_path).convert('L')
#     map_pixels = np.array(img)
#     map_height, map_width = map_pixels.shape

#     if logger:
#         logger.info(f"Map size: {map_width} x {map_height}")

#     return map_pixels, map_width, map_height, resolution, origin_xy


# class VirtualObstaclesNode(Node):
#     """
#     Reads:
#       - SLAM map (new_map.pgm + new_map.yaml)
#       - pose_data.json (Robot Position: X, Y, Yaw)
#       - centroid_data.json (Lanes Data: Left X, Right X; Centre Data: Centre X)

#     Outputs:
#       - clean_map_obs.pgm with lane lines marked as virtual obstacles.
#     """

    

#     def __init__(self):

#         super().__init__('virtual_obstacles_node')

#         # ---------- ABSOLUTE SOURCE PATHS (MATCH YOUR TREE) ----------
#         # /home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg
#         src_base_dir = os.path.join(
#             os.path.expanduser('~'),
#             'ros2', 'src', 'slam_lane_tracking_pkg', 'slam_lane_tracking_pkg'
#         )
#         maps_dir = os.path.join(src_base_dir, 'maps2')
#         scripts_dir = os.path.join(src_base_dir, 'scripts')

#         # Defaults: everything in *source* space, not install/
#         self.declare_parameter(
#             'map_path',
#             os.path.join(maps_dir, 'new_map.pgm')
#         )
#         self.declare_parameter(
#             'output_map_path',
#             os.path.join(maps_dir, 'clean_map_obs.pgm')   # output map with virtual lanes
#         )
#         self.declare_parameter(
#             'pose_file_path',
#             os.path.join(scripts_dir, 'pose_data.json')
#         )
#         self.declare_parameter(
#             'centroid_file_path',
#             os.path.join(scripts_dir, 'centroid_data.json')
#         )

#         self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
#         self.output_map_path = self.get_parameter('output_map_path').get_parameter_value().string_value
#         self.pose_file_path = self.get_parameter('pose_file_path').get_parameter_value().string_value
#         self.centroid_file_path = self.get_parameter('centroid_file_path').get_parameter_value().string_value

#         self.get_logger().info('VirtualObstaclesNode started.')
#         self.get_logger().info(f'Map PGM path     : {self.map_path}')
#         self.get_logger().info(f'Output map path  : {self.output_map_path}')
#         self.get_logger().info(f'Pose file path   : {self.pose_file_path}')
#         self.get_logger().info(f'Centroid file path: {self.centroid_file_path}')
        
#     def run(self):
#         # ---------- Load pose data ----------
#         if not os.path.exists(self.pose_file_path):
#             self.get_logger().error(f'Pose file not found: {self.pose_file_path}')
#             return

#         with open(self.pose_file_path, 'r') as f:
#             pose_data_json = json.load(f)

#         robot_positions = pose_data_json.get("Robot Position", {})
#         x_positions = robot_positions.get("X", [])
#         y_positions = robot_positions.get("Y", [])
#         yaw_positions = robot_positions.get("Yaw", [])

#         # ---------- Load centroid data ----------
#         if not os.path.exists(self.centroid_file_path):
#             self.get_logger().error(f'Centroid file not found: {self.centroid_file_path}')
#             return

#         with open(self.centroid_file_path, 'r') as f:
#             centroids_data_json = json.load(f)

#         lanes_data = centroids_data_json.get("Lanes Data", {})
#         left_line_position = lanes_data.get("Left X", [])
#         right_line_position = lanes_data.get("Right X", [])

#         centre_data = centroids_data_json.get("Centre Data", {})
#         centre_line_position = centre_data.get("Centre X", [])

#         # ---------- Pack coordinate lists ----------
#         real_world_coords = []
#         centroids_in_world_coords = []

#         for x, y, yaw, left_x, right_x, centre_x in zip(
#             x_positions, y_positions, yaw_positions,
#             left_line_position, right_line_position, centre_line_position
#         ):
#             real_world_coords.append((x, y, yaw))
#             centroids_in_world_coords.append((left_x, right_x, centre_x))

#         self.get_logger().info(f'Number of robot positions: {len(real_world_coords)}')
#         self.get_logger().info(f'Number of line points    : {len(centroids_in_world_coords)}')

#         if len(real_world_coords) == 0 or len(centroids_in_world_coords) == 0:
#             self.get_logger().warn('No pose/centroid data to process. Exiting.')
#             return

#         # ---------- Load map with metadata ----------
#         try:
#             map_pixels, map_width, map_height, map_resolution, map_origin = load_map_and_metadata(
#                 self.map_path, logger=self.get_logger()
#             )
#         except FileNotFoundError:
#             return

#         # Optional: clear original obstacles to keep only virtual ones
#         # Comment this line if you want to keep original walls.
#         map_pixels[map_pixels == 0] = 255

#         # ---------- Consecutive occurrence tracking ----------
#         consecutive_upper_line = None
#         consecutive_lower_line = None
#         upper_count = 0
#         lower_count = 0

#         # ---------- Process each pose + centroid sample ----------
#         for (real_x, real_y, pose), (L_x, R_x, C_x) in zip(real_world_coords, centroids_in_world_coords):

#             # Distances from center to left/right lanes (in meters, after scaling)
#             upper_line = round(abs((C_x - L_x)) * 0.01, 2)
#             lower_line = round(abs((C_x - R_x)) * 0.01, 2)

#             # Track consecutive values for upper_line
#             if upper_line == consecutive_upper_line:
#                 upper_count += 1
#             else:
#                 consecutive_upper_line = upper_line
#                 upper_count = 1

#             # Track consecutive values for lower_line
#             if lower_line == consecutive_lower_line:
#                 lower_count += 1
#             else:
#                 consecutive_lower_line = lower_line
#                 lower_count = 1

#             # ----- Upper line processing -----
#             if upper_count >= 3:
#                 # Clamp distance
#                 if upper_line < 0.14 or upper_line > 0.18:
#                     upper_line = 0.14

#                 real_upper_x = real_x + upper_line * math.cos(pose - 1.5708)
#                 real_upper_y = real_y + upper_line * math.sin(pose - 1.5708)

#                 map_upper_x, map_upper_y = real_world_to_map(
#                     real_upper_x, real_upper_y,
#                     map_origin, map_resolution, map_height
#                 )

#                 if 0 <= map_upper_x < map_width and 0 <= map_upper_y < map_height:
#                     map_pixels[map_upper_y, map_upper_x] = 0  # virtual obstacle pixel

#             # ----- Lower line processing -----
#             if lower_count >= 3:
#                 if lower_line < 0.14 or lower_line > 0.18:
#                     lower_line = 0.14

#                 real_lower_x = real_x + lower_line * math.cos(pose + 1.5708)
#                 real_lower_y = real_y + lower_line * math.sin(pose + 1.5708)

#                 map_lower_x, map_lower_y = real_world_to_map(
#                     real_lower_x, real_lower_y,
#                     map_origin, map_resolution, map_height
#                 )

#                 if 0 <= map_lower_x < map_width and 0 <= map_lower_y < map_height:
#                     map_pixels[map_lower_y, map_lower_x] = 0  # virtual obstacle pixel

#         # ---------- Save the modified map ----------
#         # os.makedirs(os.path.dirname(self.output_map_path), exist_ok=True)
#         # modified_img = Image.fromarray(map_pixels)
#         # modified_img.save(self.output_map_path)
        
#         # self.get_logger().info(f'Modified map saved to: {self.output_map_path}')

#         # ---------- Save the modified map ----------
#         os.makedirs(os.path.dirname(self.output_map_path), exist_ok=True)
#         modified_img = Image.fromarray(map_pixels)
#         modified_img.save(self.output_map_path)

#         self.get_logger().info(f'Modified map saved to: {self.output_map_path}')

#         # ---------- Save YAML for modified map ----------
#         yaml_output_path = self.output_map_path.replace('.pgm', '.yaml')

#         # Load original YAML so we keep resolution/origin/info
#         original_yaml_path = self.map_path.replace('.pgm', '.yaml')
#         with open(original_yaml_path, 'r') as f:
#             original_yaml = yaml.safe_load(f)

#         # Update only the "image" field to point to the new PGM file
#         original_yaml['image'] = os.path.basename(self.output_map_path)

#         # Write the new YAML
#         with open(yaml_output_path, 'w') as f:
#             yaml.dump(original_yaml, f, default_flow_style=False)

#         self.get_logger().info(f'Map YAML saved to: {yaml_output_path}')


       




# def main(args=None):
#     rclpy.init(args=args)
#     node = VirtualObstaclesNode()
#     try:
#         node.run()
#     except Exception as e:
#         node.get_logger().error(f'Exception during processing: {e}')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
import math
import json
import os
import yaml

def real_world_to_map(real_x, real_y, map_origin, map_resolution, map_height):
    pixel_x = int((real_x - map_origin[0]) / map_resolution)
    pixel_y = int(map_height - (real_y - map_origin[1]) / map_resolution)
    return pixel_x, pixel_y

def load_map_and_metadata(map_path, logger=None):
    yaml_path = map_path.replace('.pgm', '.yaml')
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Map YAML file not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    resolution = map_info['resolution']
    origin_xy = map_info['origin'][:2]

    if not os.path.exists(map_path):
        raise FileNotFoundError(f"Map PGM file not found: {map_path}")

    img = Image.open(map_path).convert('L')
    map_pixels = np.array(img)
    map_height, map_width = map_pixels.shape

    return map_pixels, map_width, map_height, resolution, origin_xy


class VirtualObstaclesNode(Node):
    def __init__(self):
        super().__init__('virtual_obstacles_node')

        src_base_dir = os.path.join(
            os.path.expanduser('~'),
            'ros2', 'src', 'slam_lane_tracking_pkg', 'slam_lane_tracking_pkg'
        )
        maps_dir = os.path.join(src_base_dir, 'maps')
        scripts_dir = os.path.join(src_base_dir, 'scripts')

        self.declare_parameter('map_path', os.path.join(maps_dir, 'new_map.pgm'))
        self.declare_parameter('output_map_path', os.path.join(maps_dir, 'clean_map_obs.pgm'))
        self.declare_parameter('pose_file_path', os.path.join(scripts_dir, 'pose_data.json'))
        self.declare_parameter('centroid_file_path', os.path.join(scripts_dir, 'centroid_data.json'))

        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.output_map_path = self.get_parameter('output_map_path').get_parameter_value().string_value
        self.pose_file_path = self.get_parameter('pose_file_path').get_parameter_value().string_value
        self.centroid_file_path = self.get_parameter('centroid_file_path').get_parameter_value().string_value

        self.get_logger().info('VirtualObstaclesNode started.')

    def run(self):
        # 1. Load Pose Data
        if not os.path.exists(self.pose_file_path):
            self.get_logger().error('Pose file missing.')
            return
        with open(self.pose_file_path, 'r') as f:
            pose_data = json.load(f)
        
        robot_pos = pose_data.get("Robot Position", {})
        x_list = robot_pos.get("X", [])
        y_list = robot_pos.get("Y", [])
        yaw_list = robot_pos.get("Yaw", [])

        # 2. Load Centroid Data
        if not os.path.exists(self.centroid_file_path):
            self.get_logger().error('Centroid file missing.')
            return
        with open(self.centroid_file_path, 'r') as f:
            cent_data = json.load(f)

        lanes = cent_data.get("Lanes Data", {})
        left_list = lanes.get("Left X", [])
        right_list = lanes.get("Right X", [])
        centre_list = cent_data.get("Centre Data", {}).get("Centre X", [])

        # 3. Load Map
        try:
            map_pixels, map_w, map_h, map_res, map_orig = load_map_and_metadata(self.map_path)
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            return

        # Clear existing obstacles (make everything free space first) - Optional
        map_pixels[map_pixels == 0] = 255

        # 4. Processing
        # We zip lists. Note: If one list is shorter, zip stops there.
        # L_x or R_x can be None now.
        
        consecutive_upper = None
        consecutive_lower = None
        upper_count = 0
        lower_count = 0

        for (rx, ry, ryaw), (Lx, Rx, Cx) in zip(zip(x_list, y_list, yaw_list), zip(left_list, right_list, centre_list)):
            
            # --- Process LEFT Lane (Yellow) ---
            if Lx is not None and Cx is not None:
                upper_line = round(abs((Cx - Lx)) * 0.01, 2)
                
                # Logic to reduce noise
                if upper_line == consecutive_upper:
                    upper_count += 1
                else:
                    consecutive_upper = upper_line
                    upper_count = 1

                if upper_count >= 3:
                    # Clamp
                    if upper_line < 0.14 or upper_line > 0.18:
                        upper_line = 0.14
                    
                    real_ux = rx + upper_line * math.cos(ryaw - 1.5708)
                    real_uy = ry + upper_line * math.sin(ryaw - 1.5708)
                    
                    mx, my = real_world_to_map(real_ux, real_uy, map_orig, map_res, map_h)
                    if 0 <= mx < map_w and 0 <= my < map_h:
                        map_pixels[my, mx] = 0

            # --- Process RIGHT Lane (White) ---
            if Rx is not None and Cx is not None:
                lower_line = round(abs((Cx - Rx)) * 0.01, 2)

                if lower_line == consecutive_lower:
                    lower_count += 1
                else:
                    consecutive_lower = lower_line
                    lower_count = 1

                if lower_count >= 3:
                    if lower_line < 0.14 or lower_line > 0.18:
                        lower_line = 0.14
                    
                    real_lx = rx + lower_line * math.cos(ryaw + 1.5708)
                    real_ly = ry + lower_line * math.sin(ryaw + 1.5708)

                    mx, my = real_world_to_map(real_lx, real_ly, map_orig, map_res, map_h)
                    if 0 <= mx < map_w and 0 <= my < map_h:
                        map_pixels[my, mx] = 0

        # 5. Save Results
        os.makedirs(os.path.dirname(self.output_map_path), exist_ok=True)
        Image.fromarray(map_pixels).save(self.output_map_path)
        self.get_logger().info(f'Modified map saved to: {self.output_map_path}')

        # Save YAML
        yaml_out = self.output_map_path.replace('.pgm', '.yaml')
        orig_yaml_path = self.map_path.replace('.pgm', '.yaml')
        
        if os.path.exists(orig_yaml_path):
            with open(orig_yaml_path, 'r') as f:
                d = yaml.safe_load(f)
            d['image'] = os.path.basename(self.output_map_path)
            with open(yaml_out, 'w') as f:
                yaml.dump(d, f)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualObstaclesNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()