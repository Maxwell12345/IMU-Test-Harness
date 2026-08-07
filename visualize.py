import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


parser = argparse.ArgumentParser()
parser.add_argument("csv_path", nargs="?", default="build/output.csv")
args = parser.parse_args()

csv_path = Path(args.csv_path)

with csv_path.open(newline="") as csv_file:
    rows = list(csv.DictReader(csv_file))

if not rows:
    raise RuntimeError(f"No data found in {csv_path}")

timestamps = [int(row["measurement_host_timestamp_ns"]) for row in rows]
start_timestamp = timestamps[0]
elapsed_seconds = [(timestamp - start_timestamp) / 1_000_000_000 for timestamp in timestamps]
gps_longitude = [float(row["gps_longitude_deg"]) for row in rows]
gps_latitude = [float(row["gps_latitude_deg"]) for row in rows]
kf_easting = [float(row["kf_easting_m"]) for row in rows]
kf_northing = [float(row["kf_northing_m"]) for row in rows]
kf_speed = [float(row["kf_speed_mps"]) for row in rows]
kf_heading = [float(row["kf_heading_deg"]) for row in rows]

figure, axes = plt.subplots(2, 2, figsize=(14, 10))

axes[0, 0].plot(gps_longitude, gps_latitude)
axes[0, 0].set_title("GPS Path")
axes[0, 0].set_xlabel("Longitude (degrees)")
axes[0, 0].set_ylabel("Latitude (degrees)")
axes[0, 0].axis("equal")
axes[0, 0].grid(True)

axes[0, 1].plot(kf_easting, kf_northing)
axes[0, 1].set_title("Kalman Filter ENU Path")
axes[0, 1].set_xlabel("Easting (m)")
axes[0, 1].set_ylabel("Northing (m)")
axes[0, 1].axis("equal")
axes[0, 1].grid(True)

axes[1, 0].plot(elapsed_seconds, kf_speed)
axes[1, 0].set_title("Kalman Filter Speed")
axes[1, 0].set_xlabel("Elapsed time (s)")
axes[1, 0].set_ylabel("Speed (m/s)")
axes[1, 0].grid(True)

axes[1, 1].plot(elapsed_seconds, kf_heading)
axes[1, 1].set_title("Kalman Filter Heading")
axes[1, 1].set_xlabel("Elapsed time (s)")
axes[1, 1].set_ylabel("Heading (degrees)")
axes[1, 1].set_ylim(0, 360)
axes[1, 1].grid(True)

figure.suptitle(csv_path.name)
figure.tight_layout()
plt.show()
