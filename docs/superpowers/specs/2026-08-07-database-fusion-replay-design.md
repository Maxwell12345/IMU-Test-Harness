# Database Fusion Replay Design

## Goal

Build a simulation entry point that replays the complete `build/imu_data_good.db` recording through the existing GPS, `IMUManager`, `RadarPositionNavigationController`, and Kalman-filter logic, then writes synchronized GPS and filter results to CSV. Preserve a production build that does not contain simulation entry-point behavior.

## Constraints

- Use C++20 and the repository's existing CMake, SQLiteCpp, Eigen, yaml-cpp, and Google Test dependencies.
- Keep all replay cursors, event state, CSV state, and replay orchestration in `src/main2.cpp`.
- Add no new private methods to existing classes.
- Add only `GetKFState()` and `GetKFCovariance()` to the public controller API.
- Fully document changed public APIs using the project Doxygen convention.
- Preserve the user's uncommitted removal of `KineticState`.
- Use IMU payload timestamps for filter measurement timing.
- Put each GPS row's database `host_timestamp_ns` into the replayed `GpsUpdate`.
- Treat `SIMULATION_MODE` only as a CMake configuration option. Do not define a C++ preprocessor symbol.
- Keep the recorded input database read-only.
- Add no runtime internet dependency.

## Build Selection

`CMakeLists.txt` will define `option(SIMULATION_MODE ... OFF)`. When OFF, the `IMUPROC` executable will use `src/main.cpp`; when ON, it will use `src/main2.cpp`. Both entry-point files will be excluded from `IMUPROC_LIB` so the library contains no `main()` implementation.

`build.sh` will always use the `build` directory. Its first argument selects the mode: exactly `s` configures `SIMULATION_MODE=ON`; a missing first argument or any other value configures it OFF. A `clean` argument remains supported, including `./build.sh s clean`. The script will receive the project-required file header.

## Production Path

The current default executable fails before application startup because `GpsManager` constructs `NmeaReader` with hard-coded `/dev/ttyTEST`, and `NmeaReader` opens that device in its constructor. The default production entry point will no longer instantiate this test-only GPS manager. A non-null IMU serial reader will retain its current production startup and shutdown behavior.

## Controller Refactor

`RadarPositionNavigationController` will expose:

```cpp
Vector6d GetKFState() const;
Matrix6d GetKFCovariance() const;
```

Both methods return locked snapshots under the existing Kalman-filter mutex. The mutex will become `mutable`; no new controller state is required.

The existing constructor and serial lifecycle helpers will tolerate a null `IMUSerialPortReader`. This permits `main2.cpp` to configure the controller without starting hardware while leaving non-null production behavior unchanged. No simulation-specific controller method will be added.

The controller will initialize its filter position as zero easting and zero northing, with the supplied latitude and longitude defining the ENU origin. Existing conversion arguments and variables will be corrected and renamed so longitude/latitude inputs and easting/northing outputs cannot be confused. Filter state and control names will describe their current roles: easting, northing, heading, speed, yaw rate, and acceleration.

## GPS Parsing

The existing private NMEA parsing functions on `NmeaReader` will become public static functions so a database sentence can be parsed without constructing serial hardware. The existing `GpsManager::BuildGpsUpdate` mapping function will become public and static and will accept the database timestamp.

For replayed GPS rows:

- `GpsUpdate::timestamp` receives `host_timestamp_ns / 1e9` seconds.
- `GpsUpdate::gpsTimestampMs` receives `host_timestamp_ns / 1e6` milliseconds after range validation.
- `GpsUpdate::receiveTime` records replay delivery time solely for the existing freshness check; it is not used for filter measurement timing.
- GPS1 RMC sentences provide latitude, longitude, validity, and course-over-ground.

## Replay Data Flow

`main2.cpp` will open `build/imu_data_good.db` with `SQLite::OPEN_READONLY`. A single ordered query will merge GPS1 RMC, linear-acceleration, rotation-vector, and delta-rotation-rate records by `host_timestamp_ns`, with deterministic tie-breaking.

The first valid GPS record establishes the controller's ENU origin. GPS events are passed through `RadarPositionNavigationController::GetGPSCallback()`. IMU events are reconstructed as the existing `Raw_Accelerometer`, `Raw_RotationVectorWAcc`, and `Raw_RotationRate` payload types and passed through `IMUManager::SensorCallback()`.

`main2.cpp` retains a non-owning `IMUManager*` before transferring the manager's ownership to the controller. Its lifetime is bounded by the controller. The controller receives a null serial reader. An in-memory `DatabaseManager` satisfies the existing persistence dependency without modifying the recorded database or leaving a second database artifact.

The existing `DatabaseManagerStats::ekfQueued` counter changes synchronously whenever the controller completes a filter step. `main2.cpp` uses that counter to emit exactly one CSV row per completed step, then obtains the locked state and covariance snapshots through the two new controller accessors.

## Timing and Filter Inputs

`IMUManager::PrepareEkfTiming()` will continue deriving `dt` from IMU payload microseconds. Replay speed and wall-clock execution time will not affect `dt`.

The filter motion model consumes forward acceleration and yaw rate. The IMU measurement mapping will use the documented vehicle-forward acceleration axis and the recorded yaw rate rather than presenting east and north acceleration as mislabeled controls. Names and Doxygen contracts will be updated to describe the actual filter interface and ENU position frame.

## CSV Output

The replay writes `build/imu_fusion_replay.csv`, replacing an older CSV of the same name. Each completed filter update produces one row containing:

- database timestamp in nanoseconds;
- GPS latitude and longitude in decimal degrees;
- optional GPS course in degrees;
- KF easting and northing in meters;
- KF speed in meters per second;
- KF heading in radians and degrees;
- all 36 covariance entries as individual `kf_covariance_<row>_<column>` columns.

The CSV uses sufficient floating-point precision for numerical analysis and reports missing GPS course as an empty field.

## Error Handling

Simulation exits with failure if the input database or configuration cannot be opened, the output CSV cannot be created, GPS timestamps cannot fit required fields, no valid initial GPS record exists, SQLite iteration fails, or the fusion path throws. Controller and database shutdown use their existing lifecycle methods and flush behavior.

## Verification

Tests will cover:

- controller state and covariance snapshots;
- null and non-null serial-reader lifecycle behavior;
- ENU origin initialization and coordinate ordering;
- NMEA parsing without serial construction;
- database-timestamp mapping into `GpsUpdate`;
- IMU payload-derived `dt`;
- corrected filter input naming and yaw-rate mapping;
- CMake production and simulation source selection;
- `build.sh` selection for `s`, no argument, and non-`s` arguments.

Final verification will build and test both CMake modes, run the complete simulation database, validate the CSV header and row count, confirm finite filter outputs, and confirm that `build/imu_data_good.db` remains unchanged.
