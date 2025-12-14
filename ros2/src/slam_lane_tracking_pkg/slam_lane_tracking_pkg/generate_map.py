#!/usr/bin/env python3
import json
import numpy as np
from PIL import Image
import math
import os

# --- Configuration ---
INPUT_MAP_PGM = 'maps.pgm'     # Your original blank/SLAM map
INPUT_MAP_YAML = 'maps.yaml'   # Your original map metadata
OUTPUT_MAP_PGM = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.pgm'     # The new map with obstacles
OUTPUT_MAP_YAML = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.yaml'   # The new metadata
POSE_FILE = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/scripts/pose_data.json'
CENTROID_FILE = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/scripts/centroid_data.json'

# Track Width Logic
# User specified 26mm, but likely meant 26cm (0.26m) given robot size.
# Half width = 0.13m.
HALF_TRACK_WIDTH = 0.13 

def load_yaml_manual(path):
    """Parses simple YAML manually to avoid dependency issues."""
    meta = {}
    with open(path, 'r') as f:
        for line in f:
            if ':' in line:
                key, val = line.split(':', 1)
                key = key.strip()
                val = val.strip()
                if key == 'origin':
                    val = val.strip('[]').split(',')
                    val = [float(v) for v in val]
                elif key == 'resolution':
                    val = float(val)
                meta[key] = val
    return meta

def save_yaml_manual(path, meta, image_filename):
    """Saves YAML with updated image filename."""
    with open(path, 'w') as f:
        for key, val in meta.items():
            if key == 'image':
                f.write(f"image: {image_filename}\n")
            elif key == 'origin':
                val_str = f"[{val[0]}, {val[1]}, {val[2]}]"
                f.write(f"{key}: {val_str}\n")
            else:
                f.write(f"{key}: {val}\n")

def ensure_numeric(arr):
    """Converts list to numpy array, handling None/Null as NaN."""
    return np.array([float(x) if x is not None else np.nan for x in arr])

def world_to_map(wx, wy, origin_x, origin_y, resolution, height):
    """Converts world coordinates (meters) to map pixel coordinates."""
    mx = int((wx - origin_x) / resolution)
    # Map Y axis is inverted relative to image (0,0 at top-left)
    my = int(height - (wy - origin_y) / resolution)
    return mx, my

def main():
    print("--- Starting Map Generation ---")

    # 1. Load Map Metadata
    if os.path.exists(INPUT_MAP_YAML):
        map_meta = load_yaml_manual(INPUT_MAP_YAML)
        print(f"Loaded metadata from {INPUT_MAP_YAML}")
    else:
        print(f"Warning: {INPUT_MAP_YAML} not found. Using defaults.")
        map_meta = {'resolution': 0.05, 'origin': [-10.0, -10.0, 0.0]}

    resolution = map_meta.get('resolution', 0.05)
    origin = map_meta.get('origin', [0, 0, 0])
    origin_x = origin[0]
    origin_y = origin[1]

    # 2. Load Base Map Image
    if os.path.exists(INPUT_MAP_PGM):
        image = Image.open(INPUT_MAP_PGM).convert("L")
        print(f"Loaded base map: {INPUT_MAP_PGM} {image.size}")
    else:
        print(f"Warning: {INPUT_MAP_PGM} not found. Creating new blank map.")
        image = Image.new("L", (2000, 2000), color=255) # 100x100m blank
        origin_x = -50.0
        origin_y = -50.0
        map_meta['origin'] = [origin_x, origin_y, 0.0]

    map_pixels = np.array(image)
    map_height, map_width = map_pixels.shape

    # 3. Load Data Files
    try:
        with open(POSE_FILE, 'r') as f:
            pose_data = json.load(f)
        with open(CENTROID_FILE, 'r') as f:
            centroid_data = json.load(f)
    except FileNotFoundError as e:
        print(f"Error: Data file not found - {e}")
        return

    # 4. Process Data
    px = np.array(pose_data['Robot Position']['X'])
    py = np.array(pose_data['Robot Position']['Y'])
    pyaw = np.array(pose_data['Robot Position']['Yaw'])

    lx = ensure_numeric(centroid_data['Lanes Data']['Left X'])
    rx = ensure_numeric(centroid_data['Lanes Data']['Right X'])
    cx = ensure_numeric(centroid_data['Centre Data']['Centre X'])

    # 5. Interpolate Pose to match Centroid Data density
    # (Pose is usually slower than Camera, so we upsample Pose)
    n_pose = len(px)
    n_cent = len(lx)
    print(f"Interpolating {n_pose} pose points to match {n_cent} centroid points...")

    t_pose = np.linspace(0, 1, n_pose)
    t_cent = np.linspace(0, 1, n_cent)

    # Unwrap yaw to handle -pi/pi transitions correctly
    pyaw_unwrapped = np.unwrap(pyaw)
    
    px_interp = np.interp(t_cent, t_pose, px)
    py_interp = np.interp(t_cent, t_pose, py)
    pyaw_interp = np.interp(t_cent, t_pose, pyaw_unwrapped)

    # 6. Draw Virtual Obstacles
    print("Projecting virtual obstacles onto map...")
    for i in range(n_cent):
        pose_x = px_interp[i]
        pose_y = py_interp[i]
        pose_yaw = pyaw_interp[i]
        
        l_val = lx[i]
        r_val = rx[i]
        c_val = cx[i]
        
        # --- Left Lane Projection ---
        # Logic: If Left Lane detected, draw wall at Fixed Offset to the RIGHT (-90 deg)
        # (Preserving your previous node's projection logic)
        if not np.isnan(l_val) and not np.isnan(c_val):
            obs_x = pose_x + HALF_TRACK_WIDTH * math.cos(pose_yaw - 1.5708)
            obs_y = pose_y + HALF_TRACK_WIDTH * math.sin(pose_yaw - 1.5708)
            
            mx, my = world_to_map(obs_x, obs_y, origin_x, origin_y, resolution, map_height)
            if 0 <= mx < map_width and 0 <= my < map_height:
                map_pixels[my, mx] = 0 # Black pixel

        # --- Right Lane Projection ---
        # Logic: If Right Lane detected, draw wall at Fixed Offset to the LEFT (+90 deg)
        if not np.isnan(r_val) and not np.isnan(c_val):
            obs_x = pose_x + HALF_TRACK_WIDTH * math.cos(pose_yaw + 1.5708)
            obs_y = pose_y + HALF_TRACK_WIDTH * math.sin(pose_yaw + 1.5708)
            
            mx, my = world_to_map(obs_x, obs_y, origin_x, origin_y, resolution, map_height)
            if 0 <= mx < map_width and 0 <= my < map_height:
                map_pixels[my, mx] = 0 # Black pixel

    # 7. Save Outputs
    output_img = Image.fromarray(map_pixels)
    output_img.save(OUTPUT_MAP_PGM)
    save_yaml_manual(OUTPUT_MAP_YAML, map_meta, OUTPUT_MAP_PGM)

    print(f"Success! Created {OUTPUT_MAP_PGM} and {OUTPUT_MAP_YAML}")

if __name__ == '__main__':
    main()