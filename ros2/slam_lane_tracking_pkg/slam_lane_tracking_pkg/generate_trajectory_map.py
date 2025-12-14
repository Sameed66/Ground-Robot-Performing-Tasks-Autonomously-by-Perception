#!/usr/bin/env python3
import json
import numpy as np
from PIL import Image, ImageDraw
import os
import yaml

# --- CONFIGURATION ---
# Absolute paths are safest
BASE_DIR = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg'
POSE_FILE = os.path.join(BASE_DIR, 'scripts/pose_data.json')
INPUT_YAML = os.path.join(BASE_DIR, 'maps/map_obs.yaml')
OUTPUT_PGM = os.path.join(BASE_DIR, 'maps/trajectory_map.pgm')
OUTPUT_YAML = os.path.join(BASE_DIR, 'maps/trajectory_map.yaml')

# Map Parameters (From your provided file)
MAP_WIDTH = 177
MAP_HEIGHT = 195
RESOLUTION = 0.05
ORIGIN = [-4.26, -5.77, 0.0]  # [x, y, yaw]

# Trajectory "Tunnel" Width
# 0.3 meters (~6 pixels) ensures the robot fits. 
# If too narrow, the costmap will see it as blocked.
PATH_THICKNESS_METERS = 0.30 
PATH_THICKNESS_PIXELS = int(PATH_THICKNESS_METERS / RESOLUTION)

def world_to_map(wx, wy, origin_x, origin_y, resolution, height):
    """Convert World (meters) -> Image Pixel (u, v)"""
    px = int((wx - origin_x) / resolution)
    # Map Y is inverted (0,0 is top-left in image)
    py = int(height - (wy - origin_y) / resolution)
    return px, py

def main():
    print("--- Generating Trajectory Tunnel Map ---")
    
    # 1. Load Pose Data
    if not os.path.exists(POSE_FILE):
        print(f"Error: {POSE_FILE} not found.")
        return

    with open(POSE_FILE, 'r') as f:
        data = json.load(f)
    
    x_list = data['Robot Position']['X']
    y_list = data['Robot Position']['Y']
    
    # 2. Create Blank Black Map (All Obstacles)
    # Mode 'L' = Grayscale (0=Black, 255=White)
    # Color 0 = Black (Obstacle)
    image = Image.new("L", (MAP_WIDTH, MAP_HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    # 3. Convert Trajectory to Pixels
    points = []
    for wx, wy in zip(x_list, y_list):
        px, py = world_to_map(wx, wy, ORIGIN[0], ORIGIN[1], RESOLUTION, MAP_HEIGHT)
        points.append((px, py))

    # 4. Draw the Trajectory as White (Free Space)
    # We draw a thick line so the robot considers it drivable space
    # Fill 254 = Free Space
    draw.line(points, fill=254, width=PATH_THICKNESS_PIXELS)
    
    # Also draw circles at vertices to smooth corners
    half_width = PATH_THICKNESS_PIXELS // 2
    for p in points:
        draw.ellipse(
            (p[0]-half_width, p[1]-half_width, p[0]+half_width, p[1]+half_width),
            fill=254
        )

    # 5. Save PGM
    image.save(OUTPUT_PGM)
    print(f"Map image saved: {OUTPUT_PGM}")

    # 6. Save YAML
    yaml_data = {
        'image': 'trajectory_map.pgm',
        'mode': 'trinary',
        'resolution': RESOLUTION,
        'origin': ORIGIN,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    with open(OUTPUT_YAML, 'w') as f:
        f.write(f"image: {yaml_data['image']}\n")
        f.write(f"mode: {yaml_data['mode']}\n")
        f.write(f"resolution: {yaml_data['resolution']}\n")
        f.write(f"origin: {yaml_data['origin']}\n")
        f.write(f"negate: {yaml_data['negate']}\n")
        f.write(f"occupied_thresh: {yaml_data['occupied_thresh']}\n")
        f.write(f"free_thresh: {yaml_data['free_thresh']}\n")
    
    print(f"Metadata saved: {OUTPUT_YAML}")

if __name__ == '__main__':
    main()