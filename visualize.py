import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv(
    "./build/output.csv",
    names=[
        "timestamp",
        "lon",
        "lat",
        "vLon",
        "vLat",
        "aLon",
        "aLat"
    ]
)

est_lon = []
est_lat = []

last_est_lon = None
last_est_lat = None
last_time = None

first_gps = True
GPS_SKIPS = 2
gps_n_skips = GPS_SKIPS

for _, row in df.iterrows():

    ts = row["timestamp"] * 1e-3

    gps_available = (
        row["lon"] != 0.0 and
        row["lat"] != 0.0
    )

    if gps_available: 
        if gps_n_skips == GPS_SKIPS:
            last_est_lon = row["lon"]
            last_est_lat = row["lat"]
            # first_gps = False
            gps_n_skips = 0
            last_time = ts
        gps_n_skips += 1

    elif last_est_lon is not None:

        dt = ts - last_time

        last_est_lon = (
            last_est_lon
            + row["vLon"] * dt
            + 0.5 * row["aLon"] * dt * dt
        )

        last_est_lat = (
            last_est_lat
            + row["vLat"] * dt
            + 0.5 * row["aLat"] * dt * dt
        )

    est_lon.append(last_est_lon)
    est_lat.append(last_est_lat)

    last_time = ts

gps_mask = (
    (df["lon"] != 0.0) &
    (df["lat"] != 0.0)
)

plt.figure(figsize=(30, 30))

plt.plot(
    df.loc[gps_mask, "lon"],
    df.loc[gps_mask, "lat"],
    ".",
    label="GPS Fixes",
    linewidth=2
)

plt.plot(
    est_lon,
    est_lat,
    "-",
    label="Dead Reckoning",
    linewidth=2
)

plt.scatter(
    df.loc[gps_mask, "lon"],
    df.loc[gps_mask, "lat"],
    s=50,
    marker="x",
    label="GPS Corrections"
)

plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("GPS vs Dead-Reckoned Path")
plt.legend()
plt.grid(True)
plt.axis("equal")

plt.savefig(
    "trajectory.png",
    dpi=1200,
    bbox_inches="tight"
)

print("Saved trajectory.png")