# data_visualization

Quick matplotlib-based sanity checks for the data captured by the
`data_collection` harness. Reads the SQLite database directly and renders two
figures used to confirm the collected data is good.

## Outputs

| File                    | Contents                                                                                     |
|-------------------------|----------------------------------------------------------------------------------------------|
| `gps_tracks.png`        | Lat/long ground tracks for `sparkfun-zed-f9p` and `vfan-ug353`, overlaid with start/end marks |
| `imu_verification.png`  | Time-series of raw accelerometer (x/y), linear acceleration (x/y), and rotation vector (i/j)  |

The GPS plot only includes the two receivers above and only rows that have a
non-null latitude/longitude fix. The IMU plot is intentionally separate from
the GPS plot.

## Usage

```bash
# Install dependencies (dev host only; not part of the deployed system)
python3 -m pip install --user -r requirements.txt

# Run against the default database (../imu_data.db) and write PNGs here
python3 visualize_data.py
```

### Options

| Flag           | Default            | Description                                                        |
|----------------|--------------------|--------------------------------------------------------------------|
| `--db`         | `../imu_data.db`   | Path to the SQLite database.                                       |
| `--outdir`     | this directory     | Where to write the PNG figures.                                   |
| `--only`       | `all`              | Limit output to `gps` or `imu`.                                   |
| `--max-points` | `200000`           | Max points per IMU series before stride subsampling; `0` = no limit. |
| `--show`       | off                | Display interactively (requires a GUI backend / `$DISPLAY`).      |

The database is opened with `PRAGMA query_only`, so running this while the
collector is writing is safe and never modifies the data.
