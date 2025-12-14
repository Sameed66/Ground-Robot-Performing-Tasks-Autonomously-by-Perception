# #!/usr/bin/env python3

# import json
# import os
# from PIL import Image
# import numpy as np

# # ---------------- PATH SETUP ---------------- #

# # Directory where THIS file lives:
# #   .../ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg
# package_dir = os.path.dirname(os.path.abspath(__file__))

# # Subfolders inside the package
# scripts_dir = os.path.join(package_dir, "scripts")
# maps_dir    = os.path.join(package_dir, "maps")

# # Input files
# clean_map_path  = os.path.join(maps_dir, "clean_map.pgm")       # you create/edit this
# pose_file_path  = os.path.join(scripts_dir, "pose_data.json")   # from odom_logger

# # Output file
# output_map_path = os.path.join(maps_dir, "clean_map_with_traj.pgm")

# # ---------------- MAP METADATA ---------------- #
# # IMPORTANT: set these from your <map>.yaml file
# map_resolution = 0.05            # meters per pixel
# map_origin = [-10.0, -10.0]      # [origin_x, origin_y] in world coords (bottom-left)


# def real_world_to_map(real_x, real_y, map_origin, map_resolution, map_height):
#     """
#     Convert world (x,y) in meters to map pixel (x,y).
#     Assumes map origin is at bottom-left in world coordinates.
#     """
#     pixel_x = int((real_x - map_origin[0]) / map_resolution)
#     pixel_y = int(map_height - (real_y - map_origin[1]) / map_resolution)
#     return pixel_x, pixel_y


# def main():
#     print("Package dir:", package_dir)
#     print("Scripts dir:", scripts_dir)
#     print("Maps dir:", maps_dir)
#     print("Using clean map:", clean_map_path)

#     # ---------- Load map ---------- #
#     if not os.path.exists(clean_map_path):
#         print(f"❌ Clean map not found: {clean_map_path}")
#         print("Create clean_map.pgm first (copy/edit new_map_obs.pgm).")
#         return

#     img = Image.open(clean_map_path)
#     map_pixels = np.array(img)
#     map_height, map_width = map_pixels.shape

#     # ---------- Load pose data ---------- #
#     if not os.path.exists(pose_file_path):
#         print(f"❌ pose_data.json not found: {pose_file_path}")
#         return

#     with open(pose_file_path, "r") as f:
#         pose_data_json = json.load(f)

#     robot_positions = pose_data_json.get("Robot Position", {})
#     X = robot_positions.get("X", [])
#     Y = robot_positions.get("Y", [])
#     Yaw = robot_positions.get("Yaw", [])

#     print(f"✅ Loaded {len(X)} poses.")

#     # ---------- Draw trajectory ---------- #
#     for x, y in zip(X, Y):
#         map_x, map_y = real_world_to_map(
#             x, y,
#             map_origin,
#             map_resolution,
#             map_height
#         )

#         if 0 <= map_x < map_width and 0 <= map_y < map_height:
#             map_pixels[map_y, map_x] = 0  # black pixel for trajectory

#     # ---------- Save result ---------- #
#     out_img = Image.fromarray(map_pixels)
#     out_img.save(output_map_path)

#     print("✅ Trajectory projected map saved to:")
#     print(f"   {output_map_path}")


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

import json
import os
from PIL import Image
import numpy as np
import yaml

BASE_DIR = "/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg"

scripts_dir = os.path.join(BASE_DIR, "scripts")
maps_dir    = os.path.join(BASE_DIR, "maps2")

# Use a map that matches new_map.yaml size & resolution
input_map_path  = os.path.join(maps_dir, "clean_map_obs.pgm")   # or new_map_obs.pgm/new_map.pgm
map_yaml_path   = os.path.join(maps_dir, "new_map.yaml")
pose_file_path  = os.path.join(scripts_dir, "pose_data.json")

output_map_path = os.path.join(maps_dir, "clean_map_with_traj.pgm")


def real_world_to_map(real_x, real_y, map_origin, map_resolution, map_height):
    pixel_x = int((real_x - map_origin[0]) / map_resolution)
    pixel_y = int(map_height - (real_y - map_origin[1]) / map_resolution)
    return pixel_x, pixel_y


def main():
    print("=== Reproject Pose on Map ===")
    print(f"Map PGM  : {input_map_path}")
    print(f"Map YAML : {map_yaml_path}")
    print(f"Pose file: {pose_file_path}")
    print()

    # Load map metadata
    if not os.path.exists(map_yaml_path):
        print(f"❌ new_map.yaml not found: {map_yaml_path}")
        return

    with open(map_yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    map_resolution = map_info['resolution']
    map_origin = map_info['origin'][:2]

    # Load map image
    if not os.path.exists(input_map_path):
        print(f"❌ Input map not found: {input_map_path}")
        return

    img = Image.open(input_map_path).convert('L')
    map_pixels = np.array(img)
    map_height, map_width = map_pixels.shape

    print(f"✅ Map size: {map_width} x {map_height}")
    print(f"✅ resolution: {map_resolution}, origin: {map_origin}")

    # Load pose data
    if not os.path.exists(pose_file_path):
        print(f"❌ pose_data.json not found: {pose_file_path}")
        return

    with open(pose_file_path, "r") as f:
        pose_data_json = json.load(f)

    robot_positions = pose_data_json.get("Robot Position", {})
    X = robot_positions.get("X", [])
    Y = robot_positions.get("Y", [])
    Yaw = robot_positions.get("Yaw", [])

    print(f"✅ Loaded {len(X)} poses from pose_data.json")
    if len(X) == 0:
        print("⚠ No poses to draw. Exiting.")
        return

    # Draw trajectory
    count_drawn = 0
    for x, y in zip(X, Y):
        map_x, map_y = real_world_to_map(
            x, y,
            map_origin,
            map_resolution,
            map_height
        )

        if 0 <= map_x < map_width and 0 <= map_y < map_height:
            map_pixels[map_y, map_x] = 0
            count_drawn += 1

    print(f"✅ Drew {count_drawn} points on the map.")

    out_img = Image.fromarray(map_pixels)
    out_img.save(output_map_path)

    print("✅ Trajectory projected map saved to:")
    print(f"   {output_map_path}")


if __name__ == "__main__":
    main()