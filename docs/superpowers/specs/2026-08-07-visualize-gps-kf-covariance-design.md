# GPS, Kalman Filter, and Covariance Visualization Design

## Scope

Update the root `visualize.py` script to display exactly two Matplotlib figures from the selected CSV file.

## Path Figure

The first figure overlays the GPS and Kalman-filter paths. GPS uses `gps_longitude_deg` for the x-axis and `gps_latitude_deg` for the y-axis. The Kalman-filter path uses `kf_easting_m` for the x-axis and `kf_northing_m` for the y-axis because those CSV fields contain longitude and latitude despite their current labels. Both series use longitude and latitude degree labels, distinct plot labels, a legend, equal axis scaling, and a grid.

## Covariance Figure

The second figure plots elapsed seconds against the trace of the full six-by-six Kalman-filter covariance matrix. Each trace value is the sum of `kf_covariance_0_0`, `kf_covariance_1_1`, `kf_covariance_2_2`, `kf_covariance_3_3`, `kf_covariance_4_4`, and `kf_covariance_5_5` from the corresponding CSV row.

## Removed Outputs

The script no longer plots KF speed, KF heading, or GPS and KF paths in separate axes. It creates no additional figures or subplots.

## Error Handling and Verification

Existing missing-file, empty-file, and missing-column behavior remains unchanged. Verification will exercise the script with a representative CSV under a noninteractive Matplotlib backend and inspect the resulting figures, axes, and plotted data. The Python source will contain no comments.
