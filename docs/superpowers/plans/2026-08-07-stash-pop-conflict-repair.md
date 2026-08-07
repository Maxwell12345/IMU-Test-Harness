# Stash-Pop Conflict Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve the stash-pop conflict while preserving the upstream quaternion/declination IMU processing, payload-derived filter timing, database GPS replay, and simulation CSV output.

**Architecture:** GPS RMC parsing will carry its measurement year into `GpsUpdate`, allowing magnetic declination to use measurement metadata instead of host execution time. `IMUManager` will calculate payload `dt` before building the new ENU-aware control vector, and `main2.cpp` will restore ordered GPS delivery and workspace-relative input/output paths.

**Tech Stack:** C++20, Eigen3, SQLiteCpp, Google Test, CMake, Bash.

## Global Constraints

- Do not run Git commands.
- Do not add private methods.
- Do not introduce a C++ `SIMULATION_MODE` preprocessor symbol.
- Use IMU payload timestamps exclusively for filter `dt`.
- Use the RMC measurement date, not `system_clock::now()`, for magnetic-declination year.
- Keep `build/imu_data_good.db` read-only.
- Preserve the new `RotateLinearAccelToTrueENU()` and `ComputeENUHeading()` processing.
- Keep simulation output at `build/output.csv` for `visualize.py`.

---

### Task 1: Carry the GPS Measurement Year Through Existing Payload Types

**Files:**
- Modify: `src/gps/NmeaMessage.hpp`
- Modify: `src/gps/NmeaReader.cpp`
- Modify: `src/GpsUpdate.hpp`
- Modify: `src/gps/GpsManager.cpp`
- Modify: `src/gps/GpsManager.hpp`
- Modify: `test/SimulationReplayTests.cpp`

**Interfaces:**
- Consumes: RMC date field `ddmmyy`, currently field index 9 in `NmeaReader::Parse()`.
- Produces: `std::optional<int> NmeaMessage::measurementYear` and `std::optional<int> GpsUpdate::measurementYear`.

- [ ] **Step 1: Extend the recorded-RMC test with the measurement year**

Add the following assertion to `SimulationReplayTest.RecordedRmcMapsDatabaseTimestampIntoGpsUpdate`:

```cpp
ASSERT_TRUE(update.measurementYear.has_value());
EXPECT_EQ(*update.measurementYear, 2026);
```

- [ ] **Step 2: Build the targeted test and verify the missing-field failure**

Run:

```bash
cmake --build build --target IMUPROC_tests -j2
```

Expected: compilation fails because `GpsUpdate` does not yet expose `measurementYear`.

- [ ] **Step 3: Add optional measurement-year fields**

Add `<optional>` to `NmeaMessage.hpp` and add the same field to both payload structs:

```cpp
std::optional<int> measurementYear;
```

Copy `measurementYear` in `GpsUpdate::operator=()`.

- [ ] **Step 4: Parse the RMC year without using host time**

In the RMC branch of `NmeaReader::Parse()`, validate the six-character date and apply the NMEA two-digit-year pivot:

```cpp
const std::string& rmcDate = f[9];
const bool validDate = rmcDate.size() == 6 &&
    std::all_of(rmcDate.begin(), rmcDate.end(), [](unsigned char character) {
        return std::isdigit(character) != 0;
    });

if (validDate) {
    const int twoDigitYear = std::stoi(rmcDate.substr(4, 2));
    msg.measurementYear = twoDigitYear >= 80 ? 1900 + twoDigitYear : 2000 + twoDigitYear;
}
```

Add `<algorithm>` and `<cctype>` includes. Map the result in `GpsManager::BuildGpsUpdate()`:

```cpp
update.measurementYear = msg.measurementYear;
```

Update the public Doxygen contract to state that RMC measurement year is propagated when present.

- [ ] **Step 5: Run the GPS replay tests**

Run:

```bash
./build/IMUPROC_tests --gtest_filter='SimulationReplayTest.*'
```

Expected: both simulation replay tests pass and the recorded sentence produces measurement year 2026.

---

### Task 2: Resolve the IMUManager Conflict as a Single Payload-Timed Flow

**Files:**
- Modify: `src/IMUManager.cpp:97-150`
- Modify: `src/IMUManager.cpp:223-254`
- Modify: `src/IMUManager.hpp:198-219`
- Modify: `test/IMUManagerTests.cpp:336-352`

**Interfaces:**
- Consumes: synchronized rotation vector, linear acceleration, rotation rate, GPS update, GPS measurement year, and payload-derived `dt`.
- Produces: two-element filter control `[forwardAcceleration, yawRate]`.

- [ ] **Step 1: Update the active IMU-vector test to the merged signature**

Replace the two-argument call with complete recorded inputs:

```cpp
const Raw_RotationVectorWAcc rotationVector{0.0F, 0.0F, 0.0F, 1.0F, 3.0F, 1'000'000ULL};
const Raw_Accelerometer linearAcceleration{1.25F, 2.5F, -0.5F, 1'000'000ULL};
const Raw_RotationRate rotationRate{0.1F, -0.2F, 0.75F, 1'000'000ULL};
GpsUpdate gpsUpdate{};
gpsUpdate.latitude = 32.6969315;
gpsUpdate.longitude = -117.2328995;

const Eigen::Matrix<double, 2, 1> control = imuManager.BuildImuMeasurementVector(
    rotationVector,
    linearAcceleration,
    rotationRate,
    gpsUpdate,
    2026,
    0.01);

EXPECT_TRUE(control.allFinite());
EXPECT_NEAR(control(1), 0.45, 1e-12);
```

The yaw expectation is `0.6 * 0.75` from the newly introduced EWMA state initialized to zero.

- [ ] **Step 2: Remove all three conflict markers and synthesize the dispatch order**

Replace the complete conflicted region with:

```cpp
double dtSeconds = PrepareEkfTiming();

if (dtSeconds <= 0.0) {
    ResetImuReadyFlags();
    return;
}

if (!gpsUpdateSnapshot->measurementYear.has_value()) {
    throw std::runtime_error("GPS measurement year is unavailable for magnetic declination");
}

Eigen::Matrix<double, 2, 1> zImu = BuildImuMeasurementVector(
    rotationVectorSnapshot,
    linearAccelerationSnapshot,
    rotationRateSnapshot,
    *gpsUpdateSnapshot,
    *gpsUpdateSnapshot->measurementYear,
    dtSeconds);
```

This preserves the stashed first-bundle timing baseline and the upstream quaternion/declination calculation. Do not restore `GetCurrentYear()` because it is absent and would make replay results depend on execution date.

- [ ] **Step 3: Fix the merged implementation return value**

At the end of `BuildImuMeasurementVector()`, return the variable the method actually constructs:

```cpp
return imuVector;
```

- [ ] **Step 4: Bring the header contract in line with the six inputs**

Document `rv`, `la`, `rr`, `gps`, `currentYear`, and `dt` with mandatory `[in]` tags. State that `currentYear` is the RMC measurement year and `dt` is seconds derived from IMU payload microseconds. Keep the existing private method signature; add no method.

- [ ] **Step 5: Scan for conflict remnants and undefined merge names**

Run:

```bash
rg -n '^(<<<<<<<|=======|>>>>>>>)|GetCurrentYear|return imuControl' src test
```

Expected: no matches.

- [ ] **Step 6: Build and run the focused IMU tests**

Run:

```bash
cmake --build build --target IMUPROC_tests -j2
./build/IMUPROC_tests --gtest_filter='IMUManagerTest.*:IMUGPSFusionKFTest.*'
```

Expected: the active IMU mapping, payload timing, non-monotonic timing, and KF semantic tests pass.

---

### Task 3: Restore Complete GPS/IMU Database Replay

**Files:**
- Modify: `src/main2.cpp:39-42`
- Modify: `src/main2.cpp:255-270`
- Modify: `docs/superpowers/specs/2026-08-07-database-fusion-replay-design.md`

**Interfaces:**
- Consumes: `build/imu_data_good.db`, `build/WMM.COF`, and ordered RMC/IMU rows.
- Produces: `build/output.csv`.

- [ ] **Step 1: Restore paths that exist when the executable is launched from the repository root**

Set:

```cpp
constexpr const char* INPUT_DATABASE_PATH = "build/imu_data_good.db";
constexpr const char* OUTPUT_CSV_PATH = "build/output.csv";
constexpr const char* CONFIG_PATH = "config.yaml";
constexpr const char* MAGNETIC_MODEL_PATH = "build/WMM.COF";
```

The current `./imu_data_good.db` and `./WMM.COF` paths do not exist; `build.sh` copies the magnetic model into `build/WMM.COF`.

- [ ] **Step 2: Re-enable GPS event delivery**

Restore the currently commented GPS block before IMU event reconstruction:

```cpp
if (eventType == ReplayEventType::Gps) {
    const NmeaMessage message = NmeaReader::Parse(eventQuery.getColumn(9).getString());
    const GpsUpdate gpsUpdate = GpsManager::BuildGpsUpdate(message, hostTimestampNs);

    if (gpsUpdate.valid && gpsUpdate.latitude != 0.0 && gpsUpdate.longitude != 0.0) {
        latestGps = gpsUpdate;
        latestGpsTimestampNs = hostTimestampNs;
        gpsCallback(latestGps);
    }

    continue;
}
```

Without this block, the entire recording uses only the initial GPS position and measurement year.

- [ ] **Step 3: Align the replay design with the supported CSV filename**

Change references to `build/imu_fusion_replay.csv` in the design document to `build/output.csv`. Keep the recorded database path as `build/imu_data_good.db`.

- [ ] **Step 4: Build simulation mode**

Run:

```bash
./build.sh s
```

Expected: `IMUPROC_LIB`, `IMUPROC`, and `IMUPROC_tests` compile with no conflict-marker, signature, undefined-name, or return-variable errors.

---

### Task 4: End-to-End Verification

**Files:**
- Verify: `build/imu_data_good.db`
- Verify: `build/output.csv`
- Verify: `build/IMUPROC`

**Interfaces:**
- Produces: evidence that both build modes work and replay output is numerically usable.

- [ ] **Step 1: Record the input hash and run the complete replay**

Run:

```bash
sha256sum build/imu_data_good.db
./build/IMUPROC
sha256sum build/imu_data_good.db
```

Expected: the executable exits zero and both database hashes match.

- [ ] **Step 2: Validate the generated CSV**

Run:

```bash
awk -F, 'NR==1 { columns=NF; next } { rows++; if (NF != columns) badWidth++; for (i=6; i<=NF; i++) { value=tolower($i); if (value == "" || value ~ /nan/ || value ~ /inf/) nonfinite++ } } END { print "columns=" columns; print "rows=" rows; print "bad_width=" (badWidth+0); print "nonfinite=" (nonfinite+0); if (columns != 46 || rows == 0 || badWidth || nonfinite) exit 1 }' build/output.csv
```

Expected: 46 columns, at least one data row, zero malformed rows, and zero non-finite KF/covariance fields.

- [ ] **Step 3: Run feature-focused tests**

Run:

```bash
./build/IMUPROC_tests --gtest_filter='SimulationReplayTest.*:IMUManagerTest.*:RadarPositionNavigationControllerTest.*:IMUGPSFusionKFTest.*'
```

Expected: all selected replay, timing, controller, origin-reset, and filter tests pass.

- [ ] **Step 4: Run the repository-wide test suite and classify failures**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected: no new failure in a file changed by this repair. Report the existing serial-protocol and magnetic-heading failures separately if they remain.

- [ ] **Step 5: Verify production mode and restore simulation as the active binary**

Run:

```bash
./build.sh
rg '^SIMULATION_MODE:' build/CMakeCache.txt
./build.sh s
rg '^SIMULATION_MODE:' build/CMakeCache.txt
```

Expected: the first cache value is `OFF`, the second is `ON`, and both modes compile.
