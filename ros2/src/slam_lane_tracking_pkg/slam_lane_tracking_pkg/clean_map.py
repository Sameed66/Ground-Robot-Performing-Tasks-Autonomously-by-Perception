# #!/usr/bin/env python3
# import numpy as np
# from PIL import Image
# import os

# # --- Configuration ---
# INPUT_PGM = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.pgm'
# INPUT_YAML = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.yaml'
# OUTPUT_PGM = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/final_map.pgm'
# OUTPUT_YAML = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/final_map.yaml'

# def main():
#     print(f"Cleaning {INPUT_PGM}...")

#     # 1. Load and Clean Image
#     # Open image and convert to Grayscale
#     img = Image.open(INPUT_PGM).convert("L")
#     data = np.array(img)

#     # Clean Logic:
#     # Pixels < 65 are Obstacles (0)
#     # Everything else (Gray/Unknown/White) becomes Free (254)
#     # This removes the "fog of war" gray areas.
#     cleaned_data = np.where(data < 65, 0, 254).astype(np.uint8)

#     # Save the new clean map
#     Image.fromarray(cleaned_data).save(OUTPUT_PGM)
#     print(f"Saved cleaned map to: {OUTPUT_PGM}")

#     # 2. Generate New YAML
#     # We read the origin/resolution from the old one but fix the image path.
#     origin_line = "origin: [0.0, 0.0, 0.0]" # Default
#     res_line = "resolution: 0.05"
    
#     if os.path.exists(INPUT_YAML):
#         with open(INPUT_YAML, 'r') as f:
#             for line in f:
#                 if line.strip().startswith('origin:'):
#                     origin_line = line.strip()
#                 if line.strip().startswith('resolution:'):
#                     res_line = line.strip()

#     # Write clean YAML with relative path
#     with open(OUTPUT_YAML, 'w') as f:
#         f.write(f"image: {OUTPUT_PGM}\n")
#         f.write("mode: trinary\n")
#         f.write(f"{res_line}\n")
#         f.write(f"{origin_line}\n")
#         f.write("negate: 0\n")
#         f.write("occupied_thresh: 0.65\n")
#         f.write("free_thresh: 0.25\n")
    
#     print(f"Saved metadata to: {OUTPUT_YAML}")

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
print("DEBUG: Script has started...")  # <--- ADD THIS LINE HERE

import numpy as np
from PIL import Image
import os

# --- Configuration ---
# (Keep your absolute paths exactly as they are)
INPUT_PGM = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.pgm'
INPUT_YAML = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/map_obs.yaml'
OUTPUT_PGM = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/final_map.pgm'
OUTPUT_YAML = '/home/masters/ros2/src/slam_lane_tracking_pkg/slam_lane_tracking_pkg/maps/final_map.yaml'

def main():
    print(f"Cleaning {INPUT_PGM}...")
    
    # Check if input file exists
    if not os.path.exists(INPUT_PGM):
        print(f"ERROR: Input file not found at {INPUT_PGM}")
        return

    # 1. Load and Clean Image
    try:
        img = Image.open(INPUT_PGM).convert("L")
        print(f"Image loaded. Size: {img.size}")
    except Exception as e:
        print(f"Error opening image: {e}")
        return

    data = np.array(img)

    # Clean Logic:
    # Pixels < 65 are Obstacles (0)
    # Everything else (Gray/Unknown/White) becomes Free (254)
    cleaned_data = np.where(data < 65, 0, 254).astype(np.uint8)

    # Save the new clean map
    try:
        Image.fromarray(cleaned_data).save(OUTPUT_PGM)
        print(f"Saved cleaned map to: {OUTPUT_PGM}")
    except Exception as e:
        print(f"Error saving map: {e}")

    # 2. Generate New YAML
    origin_line = "origin: [0.0, 0.0, 0.0]"
    res_line = "resolution: 0.05"
    
    if os.path.exists(INPUT_YAML):
        with open(INPUT_YAML, 'r') as f:
            for line in f:
                if line.strip().startswith('origin:'):
                    origin_line = line.strip()
                if line.strip().startswith('resolution:'):
                    res_line = line.strip()

    with open(OUTPUT_YAML, 'w') as f:
        f.write(f"image: {OUTPUT_PGM}\n")
        f.write("mode: trinary\n")
        f.write(f"{res_line}\n")
        f.write(f"{origin_line}\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")
    
    print(f"Saved metadata to: {OUTPUT_YAML}")

if __name__ == '__main__':
    main()