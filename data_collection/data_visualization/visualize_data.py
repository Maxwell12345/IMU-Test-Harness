"""
*******************************************************************************
* File:             visualize_data.py
*
* Author:           Patrick Sherlund
* Organization:     Marine Corps Software Factory
* Created On:       06/11/26
* Description:      Reads collected GPS and IMU samples from the SQLite
*                   database produced by the data_collection harness and
*                   renders matplotlib figures used to visually verify that
*                   the captured data is good.
*
*                   Produces:
*                     - gps_tracks.png:       lat/long ground tracks for the
*                                             sparkfun-zed-f9p and vfan-ug353
*                                             receivers, overlaid on one plot.
*                     - imu_verification.png: time-series of acceleration
*                                             (raw accelerometer and
*                                             gravity-removed linear
*                                             acceleration) and rotation-vector
*                                             quaternion components.
*******************************************************************************
"""

import argparse
import math
import os
import sqlite3
from pathlib import Path

import matplotlib

# Use a headless-safe backend unless an interactive display is available.
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# The two (and only two) GPS receivers that should be visualized.
GPS_DEVICE_IDS = ("sparkfun-zed-f9p", "vfan-ug353")

# Per-device line styling. The second device is drawn dashed and on top so the
# first device's solid line remains visible where the two tracks coincide.
GPS_STYLES = {
    "sparkfun-zed-f9p": {"color": "tab:blue", "linestyle": "-", "lw": 1.4, "zorder": 2},
    "vfan-ug353": {"color": "tab:orange", "linestyle": (0, (6, 4)), "lw": 1.1, "zorder": 3},
}

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DB = SCRIPT_DIR.parent / "imu_data.db"


def connect_db(db_path: Path) -> sqlite3.Connection:
    """Open the database in a read-only posture that is WAL-compatible.

    A plain connection is used (rather than mode=ro) so that reads work while
    the collector may still be writing in WAL mode; PRAGMA query_only then
    guarantees this process never modifies the database.
    """
    conn = sqlite3.connect(str(db_path))
    conn.execute("PRAGMA query_only = ON;")
    return conn


def load_gps(conn: sqlite3.Connection, device_ids) -> pd.DataFrame:
    placeholders = ",".join("?" for _ in device_ids)
    query = (
        "SELECT timestamp_ns, device_id, latitude, longitude "
        "FROM gps "
        "WHERE latitude IS NOT NULL AND longitude IS NOT NULL "
        f"AND device_id IN ({placeholders}) "
        "ORDER BY timestamp_ns"
    )
    return pd.read_sql_query(query, conn, params=list(device_ids))


def load_imu_table(conn: sqlite3.Connection, table: str, columns) -> pd.DataFrame:
    cols = ", ".join(["timestamp_ns", *columns])
    return pd.read_sql_query(
        f"SELECT {cols} FROM {table} ORDER BY timestamp_ns", conn
    )


def _stride(n: int, max_points: int) -> int:
    if max_points and max_points > 0 and n > max_points:
        return int(math.ceil(n / max_points))
    return 1


def print_gps_summary(gps: pd.DataFrame) -> None:
    for device in GPS_DEVICE_IDS:
        d = gps[gps["device_id"] == device]
        if d.empty:
            print(f"  {device}: 0 fixes")
            continue
        span_s = (d["timestamp_ns"].iloc[-1] - d["timestamp_ns"].iloc[0]) / 1e9
        print(
            f"  {device}: {len(d)} fixes  "
            f"lat[{d.latitude.min():.6f}, {d.latitude.max():.6f}]  "
            f"lon[{d.longitude.min():.6f}, {d.longitude.max():.6f}]  "
            f"span {span_s:.1f}s"
        )


def plot_gps(gps: pd.DataFrame, outdir: Path):
    fig, ax = plt.subplots(figsize=(11, 9))

    if gps.empty:
        print("  [warn] no GPS fixes found for the requested devices")
        return fig

    mean_lat = float(gps["latitude"].mean())
    labeled_endpoints = False

    for device in GPS_DEVICE_IDS:
        d = gps[gps["device_id"] == device]
        if d.empty:
            print(f"  [warn] no GPS fixes for {device}")
            continue

        lon = d["longitude"].to_numpy()
        lat = d["latitude"].to_numpy()
        style = GPS_STYLES[device]

        ax.plot(
            lon, lat, color=style["color"], linestyle=style["linestyle"],
            lw=style["lw"], alpha=0.9, zorder=style["zorder"],
            label=f"{device}  ({len(d)} fixes)",
        )
        ax.scatter(
            lon[0], lat[0], marker="o", s=70, facecolor="lime",
            edgecolor="black", lw=1.0, zorder=6,
            label="Start" if not labeled_endpoints else None,
        )
        ax.scatter(
            lon[-1], lat[-1], marker="X", s=80, facecolor="red",
            edgecolor="black", lw=1.0, zorder=6,
            label="End" if not labeled_endpoints else None,
        )
        labeled_endpoints = True

    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("GPS Ground Tracks - sparkfun-zed-f9p vs vfan-ug353")
    ax.grid(True, alpha=0.3)
    ax.ticklabel_format(useOffset=False, style="plain")
    ax.tick_params(axis="x", labelrotation=30)

    # Correct the aspect ratio so a degree of longitude and a degree of
    # latitude cover equal ground distance at this latitude.
    try:
        ax.set_aspect(1.0 / math.cos(math.radians(mean_lat)))
    except Exception:
        ax.set_aspect("equal")

    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), framealpha=0.9)

    out = outdir / "gps_tracks.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  wrote {out}")
    return fig


def plot_imu(panels, outdir: Path, max_points: int):
    panels = [p for p in panels if p[1] is not None and not p[1].empty]
    if not panels:
        print("  [warn] no IMU data to plot")
        return None

    t0 = min(int(p[1]["timestamp_ns"].iloc[0]) for p in panels)
    n = len(panels)
    fig, axes = plt.subplots(n, 1, figsize=(13, 3.2 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, (title, d, series, ylabel) in zip(axes, panels):
        ts = d["timestamp_ns"].to_numpy()
        stride = _stride(ts.size, max_points)
        t = (ts[::stride] - t0) / 1e9

        for col, label in series:
            ax.plot(
                t, d[col].to_numpy()[::stride],
                lw=0.5, label=label, rasterized=True,
            )

        ax.set_title(title, loc="left", fontsize=11)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=3, framealpha=0.9)
        if stride > 1:
            ax.text(
                0.99, 0.03, f"subsampled 1/{stride}",
                transform=ax.transAxes, ha="right", va="bottom",
                fontsize=8, color="gray",
            )

    axes[-1].set_xlabel("Time since session start (s)")
    fig.suptitle("IMU Data Verification", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    out = outdir / "imu_verification.png"
    fig.savefig(out, dpi=150)
    print(f"  wrote {out}")
    return fig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize collected GPS and IMU data for quality checks.",
    )
    parser.add_argument(
        "--db", default=str(DEFAULT_DB),
        help=f"Path to the SQLite database (default: {DEFAULT_DB}).",
    )
    parser.add_argument(
        "--outdir", default=str(SCRIPT_DIR),
        help="Directory to write the PNG figures into (default: this script's directory).",
    )
    parser.add_argument(
        "--max-points", type=int, default=200_000,
        help="Max points per IMU series before stride subsampling; 0 = no limit (default: 200000).",
    )
    parser.add_argument(
        "--only", choices=("all", "gps", "imu"), default="all",
        help="Limit output to GPS or IMU figures (default: all).",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display the figures interactively (requires a GUI backend).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    db_path = Path(args.db).resolve()
    if not db_path.exists():
        raise SystemExit(f"database not found: {db_path}")

    outdir = Path(args.outdir).resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    print(f"reading {db_path}")
    conn = connect_db(db_path)
    try:
        if args.only in ("all", "gps"):
            print("GPS:")
            gps = load_gps(conn, GPS_DEVICE_IDS)
            print_gps_summary(gps)
            plot_gps(gps, outdir)

        if args.only in ("all", "imu"):
            print("IMU:")
            raw = load_imu_table(conn, "accelerometer", ["acc_x", "acc_y", "acc_z"])
            lin = load_imu_table(conn, "linear_acceleration", ["lin_acc_x", "lin_acc_y", "lin_acc_z"])
            rot = load_imu_table(conn, "rotation_vector", ["quat_i", "quat_j", "quat_k"])
            print(f"  accelerometer: {len(raw)} rows")
            print(f"  linear_acceleration: {len(lin)} rows")
            print(f"  rotation_vector: {len(rot)} rows")

            panels = [
                (
                    "Raw accelerometer (acc_x, acc_y, acc_z) - includes gravity",
                    raw,
                    [("acc_x", "acc_x"), ("acc_y", "acc_y"), ("acc_z", "acc_z")],
                    "Accel (m/s^2)",
                ),
                (
                    "Linear acceleration (lin_acc_x, lin_acc_y, lin_acc_z) - gravity removed",
                    lin,
                    [("lin_acc_x", "lin_acc_x"), ("lin_acc_y", "lin_acc_y"), ("lin_acc_z", "lin_acc_z")],
                    "Accel (m/s^2)",
                ),
                (
                    "Rotation vector (quat_i, quat_j, quat_k)",
                    rot,
                    [("quat_i", "quat_i (x)"), ("quat_j", "quat_j (y)"), ("quat_k", "quat_k (z)")],
                    "Quaternion",
                ),
            ]
            plot_imu(panels, outdir, args.max_points)
    finally:
        conn.close()

    if args.show:
        plt.show()

    print("done.")


if __name__ == "__main__":
    main()
