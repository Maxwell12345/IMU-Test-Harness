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
kf_longitude = [float(row["kf_easting_m"]) for row in rows]
kf_latitude = [float(row["kf_northing_m"]) for row in rows]
covariance_trace = [
    sum(float(row[f"kf_covariance_{index}_{index}"]) for index in range(6))
    for row in rows
]

path_figure, path_axes = plt.subplots(figsize=(10, 8))
path_axes.plot(gps_longitude, gps_latitude, label="GPS")
path_axes.plot(kf_longitude, kf_latitude,marker="o", markersize=1, label="Kalman Filter")
path_axes.set_title(f"GPS and Kalman Filter Path — {csv_path.name}")
path_axes.set_xlabel("Longitude (degrees)")
path_axes.set_ylabel("Latitude (degrees)")
path_axes.axis("equal")
path_axes.grid(True)
path_axes.legend()
path_figure.tight_layout()

covariance_figure, covariance_axes = plt.subplots(figsize=(10, 6))
covariance_axes.plot(elapsed_seconds, covariance_trace)
covariance_axes.set_title(f"Kalman Filter Covariance Trace — {csv_path.name}")
covariance_axes.set_xlabel("Elapsed time (s)")
covariance_axes.set_ylabel("Covariance trace")
covariance_axes.grid(True)
covariance_figure.tight_layout()
plt.show()
