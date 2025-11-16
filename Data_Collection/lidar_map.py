#!/usr/bin/env python3
import os
import time
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
from rplidar import RPLidar

# -------------------------------------------------------
# Configuration
# -------------------------------------------------------
PORT_NAME = '/dev/ttyUSB0'
OUTPUT_DIR = os.path.expanduser('~/Desktop/LiDAR_Maps')
os.makedirs(OUTPUT_DIR, exist_ok=True)

SCAN_COUNT = 3  # number of full 360° scans to average

# -------------------------------------------------------
# Connect to LiDAR
# -------------------------------------------------------
lidar = RPLidar(PORT_NAME)
print("✅ Connected to RPLiDAR on", PORT_NAME)

time.sleep(1)
lidar.clean_input()

# -------------------------------------------------------
# Data Collection
# -------------------------------------------------------
angles = []
distances = []

print(f"📡 Collecting {SCAN_COUNT} full 360° scans...")
scan_counter = 0

try:
    for scan in lidar.iter_scans(max_buf_meas=500):
        for (_, angle, distance) in scan:
            if distance > 0:
                angles.append(angle)
                distances.append(distance)
        scan_counter += 1
        print(f"  → Completed scan {scan_counter}/{SCAN_COUNT}")
        if scan_counter >= SCAN_COUNT:
            break
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    lidar.stop()
    lidar.disconnect()
    print("🔌 LiDAR disconnected.")

# -------------------------------------------------------
# Data Conversion
# -------------------------------------------------------
angles_rad = np.deg2rad(angles)
x = distances * np.cos(angles_rad)
y = distances * np.sin(angles_rad)

# -------------------------------------------------------
# Save CSV
# -------------------------------------------------------
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(OUTPUT_DIR, f"lidar_scan_{timestamp}.csv")

df = pd.DataFrame({'angle_deg': angles, 'distance_mm': distances, 'x_mm': x, 'y_mm': y})
df.to_csv(csv_path, index=False)
print(f"💾 Saved scan data → {csv_path}")

# -------------------------------------------------------
# Generate 2D Map Image
# -------------------------------------------------------
plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=2, c='black')
plt.title("2D LiDAR Map")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.axis('equal')
plt.grid(True)

png_path = os.path.join(OUTPUT_DIR, f"lidar_map_{timestamp}.png")
plt.savefig(png_path, dpi=300)
plt.close()
print(f"🗺️  Saved 2D map image → {png_path}")

print("✅ Done! Both PNG and CSV saved successfully.")

