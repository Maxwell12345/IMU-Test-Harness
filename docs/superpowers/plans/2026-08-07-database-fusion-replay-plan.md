# Database Fusion Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a CMake-selectable database replay executable that drives recorded GPS and IMU payloads through the existing manager, controller, and Kalman filter and writes synchronized results to CSV.

**Architecture:** A CMake-only `SIMULATION_MODE` option selects either `main.cpp` or `main2.cpp`. Existing parsing, manager, controller, filter, and database classes remain the processing path; replay iteration and CSV state live only in `main2.cpp`.

**Tech Stack:** C++20, CMake, Bash, Eigen3, SQLiteCpp, yaml-cpp, Google Test.

## Global Constraints

- Preserve the user's uncommitted `KineticState` removal.
- Do not run Git commands.
- Add no new private methods.
- Add no replay state outside `src/main2.cpp`.
- Add only `GetKFState()` and `GetKFCovariance()` to the controller's public API.
- Use IMU payload timestamps for KF measurement timing.
- Use database `host_timestamp_ns` for replayed GPS timestamps and cross-stream ordering.
- Keep `build/imu_data_good.db` read-only.
- Use `SIMULATION_MODE` as a CMake argument only, never as a C++ preprocessor symbol.
- Default builds must use the production entry point and must not instantiate the hard-coded `/dev/ttyTEST` GPS reader.
- New source and shell files must use the project file header, and public APIs must use the project Doxygen format.

---

### Task 1: CMake and Build-Script Mode Selection

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `build.sh`

**Interfaces:**
- Consumes: first shell argument and CMake cache variable `SIMULATION_MODE`.
- Produces: `IMUPROC` from `src/main.cpp` when OFF and `src/main2.cpp` when ON.

- [ ] **Step 1: Record the current OFF and `s` behavior**

Run the script without changing source and confirm that the current script treats `s` as a build directory. This behavior is already evidenced by the generated untracked `s/` directory and is not repeated destructively.

- [ ] **Step 2: Add CMake entry-point selection**

Implement the following structure:

```cmake
option(SIMULATION_MODE "Build the recorded-sensor simulation entry point" OFF)

file(GLOB_RECURSE PROJ_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
list(FILTER PROJ_SRC EXCLUDE REGEX "/main(2)?\\.cpp$")

if(SIMULATION_MODE)
    set(IMUPROC_MAIN "${CMAKE_SOURCE_DIR}/src/main2.cpp")
else()
    set(IMUPROC_MAIN "${CMAKE_SOURCE_DIR}/src/main.cpp")
endif()

add_executable(${PROJECT_NAME} ${IMUPROC_MAIN})
```

- [ ] **Step 3: Refactor `build.sh` argument handling**

Use a fixed `build` directory, select ON only for an exact first argument `s`, retain `clean` scanning, pass `-DSIMULATION_MODE`, and copy `WMM.COF` through `BUILD_DIR`:

```bash
BUILD_DIR="build"
SIMULATION_MODE="OFF"
CLEAN=0

if [[ "${1:-}" == "s" ]]; then
    SIMULATION_MODE="ON"
fi

for arg in "$@"; do
    case "$arg" in
        clean|--clean)
            CLEAN=1
            ;;
    esac
done
```

- [ ] **Step 4: Verify CMake selection syntax**

Run:

```bash
bash -n build.sh
cmake -S . -B /tmp/imu-off-build -DSIMULATION_MODE=OFF
cmake -S . -B /tmp/imu-on-build -DSIMULATION_MODE=ON
```

Expected: shell syntax succeeds and both CMake configurations select one entry point without defining a C++ `SIMULATION_MODE` macro.

---

### Task 2: Controller Snapshots and ENU-Safe Lifecycle

**Files:**
- Modify: `src/RadarPositionNavigationController.hpp`
- Modify: `src/RadarPositionNavigationController.cpp`
- Modify: `test/RadarPositionNavigationControllerTests.cpp`

**Interfaces:**
- Produces: `Vector6d GetKFState() const` and `Matrix6d GetKFCovariance() const`.
- Preserves: existing constructor and `StartAndConfigureRadarPNT(double, double)` signatures.

- [ ] **Step 1: Add failing controller tests**

Add active Google Tests that construct an in-memory `DatabaseManager`, an `IMUManager` using `test/WMM.COF`, and a controller with a null serial reader:

```cpp
TEST(RadarPositionNavigationControllerTest, StateAndCovarianceAccessorsReturnLockedSnapshots) {
    auto database = std::make_shared<DatabaseManager>(":memory:");
    auto manager = std::make_unique<IMUManager>(database, TEST_DATA_DIR "/WMM.COF");
    RadarPositionNavigationController controller(TestKalmanValues(), database, nullptr, std::move(manager));

    controller.StartAndConfigureRadarPNT(32.6969315, -117.2328995);

    const Vector6d state = controller.GetKFState();
    const Matrix6d covariance = controller.GetKFCovariance();
    EXPECT_DOUBLE_EQ(state(0), 0.0);
    EXPECT_DOUBLE_EQ(state(1), 0.0);
    EXPECT_DOUBLE_EQ(covariance(0, 0), 25.0);
    EXPECT_DOUBLE_EQ(covariance(1, 1), 25.0);
}
```

- [ ] **Step 2: Run the targeted test and verify failure**

Run the existing test build. Expected: compilation fails because the getters do not exist and the constructor dereferences the null reader.

- [ ] **Step 3: Implement documented accessors and nullable serial behavior**

Add full Doxygen contracts, make the existing KF mutex mutable, lock it in both accessors, and guard existing serial install/start/stop calls:

```cpp
Vector6d RadarPositionNavigationController::GetKFState() const {
    std::lock_guard<std::mutex> lock(m_kFUpdateMutex);
    return m_latestX;
}

Matrix6d RadarPositionNavigationController::GetKFCovariance() const {
    std::lock_guard<std::mutex> lock(m_kFUpdateMutex);
    return m_latestP;
}
```

- [ ] **Step 4: Correct ENU initialization and naming**

Set the origin from `initialLatitude` and `initialLongitude`, initialize the state as `{0, 0, heading, speed, yawRate, acceleration}`, initialize `m_hasOrigin`, and rename local `E`, `N`, `lat`, `lon`, `x`, and vector parameters to unambiguous ENU/geodetic names. Correct the current longitude/latitude ordering in `ConvertGPSToENU` calls without adding helpers or state.

- [ ] **Step 5: Run controller tests**

Expected: null-reader start/stop is safe, the initial state begins at zero ENU, covariance snapshots match initialization, and destruction remains idempotent.

---

### Task 3: Reusable Database GPS Parsing

**Files:**
- Modify: `src/gps/NmeaReader.hpp`
- Modify: `src/gps/NmeaReader.cpp`
- Modify: `src/gps/GpsManager.hpp`
- Modify: `src/gps/GpsManager.cpp`
- Create: `test/SimulationReplayTests.cpp`

**Interfaces:**
- Produces: public static `NmeaReader::Parse`, `NmeaReader::ValidChecksum`, and `NmeaReader::ParseDeg` using their existing signatures.
- Produces: public static `GpsManager::BuildGpsUpdate(const NmeaMessage&, uint64_t databaseTimestampNs)`.

- [ ] **Step 1: Write failing parsing and timestamp tests**

Use a valid recorded RMC sentence and database timestamp:

```cpp
TEST(SimulationReplayTest, RecordedRmcMapsDatabaseTimestampIntoGpsUpdate) {
    const auto message = NmeaReader::Parse(
        "$GNRMC,150729.00,A,3241.81589,N,11713.97397,W,17.870,165.40,050826,,,D,V*2C");
    const GpsUpdate update = GpsManager::BuildGpsUpdate(message, 1060413146719ULL);

    EXPECT_TRUE(update.valid);
    EXPECT_NEAR(update.latitude, 32.6969315, 1e-7);
    EXPECT_NEAR(update.longitude, -117.2328995, 1e-7);
    EXPECT_NEAR(update.timestamp, 1060.413146719, 1e-9);
    EXPECT_EQ(update.gpsTimestampMs, 1060413U);
    ASSERT_TRUE(update.heading.has_value());
    EXPECT_NEAR(*update.heading, 165.40, 1e-12);
}
```

- [ ] **Step 2: Run targeted test and verify compilation failure**

Expected: parsing is inaccessible and `BuildGpsUpdate` lacks the timestamp parameter.

- [ ] **Step 3: Make existing parser methods static and public**

Move the three declarations without duplicating their implementation. Preserve serial callback behavior by calling the static parser from the existing callback.

- [ ] **Step 4: Refactor GPS mapping**

Map the database timestamp exactly:

```cpp
update.receiveTime = std::chrono::steady_clock::now();
update.timestamp = static_cast<double>(databaseTimestampNs) / 1'000'000'000.0;
update.gpsTimestampMs = static_cast<uint32_t>(databaseTimestampNs / 1'000'000ULL);
```

Throw `std::out_of_range` if the millisecond value exceeds `uint32_t`. Document the exception and timestamp units.

- [ ] **Step 5: Run GPS replay tests**

Expected: checksum, coordinates, heading, validity, and timestamp mappings match the recorded row without opening `/dev/ttyTEST`.

---

### Task 4: Filter Input Contract and Payload Timing

**Files:**
- Modify: `src/imu_data.hpp`
- Modify: `src/IMUManager.hpp`
- Modify: `src/IMUManager.cpp`
- Modify: `src/IMUGPSFusionKF.hpp`
- Modify: `src/IMUGPSFusionKF.cpp`
- Modify: `test/IMUManagerTests.cpp`
- Modify: `test/test_IMUGPSFusionKF.cpp`

**Interfaces:**
- Consumes: forward acceleration in meters per second squared and yaw rate in radians per second.
- Preserves: both `IMUGPSFusionKF::Step` public signatures.

- [ ] **Step 1: Add failing IMU mapping and timing tests**

Activate focused friend tests showing that payload microseconds determine `dt` and that the two-value IMU control contains forward acceleration and `d_yaw`.

- [ ] **Step 2: Run targeted tests and verify the current mapping fails**

Expected: the current implementation returns east/north acceleration and ignores rotation rate.

- [ ] **Step 3: Correct the mapping and names**

Rename the typo `d_raw` to `d_yaw`. Build the filter control as:

```cpp
Eigen::Matrix<double, 2, 1> imuControl = {
    static_cast<double>(linearAcceleration.y),
    static_cast<double>(rotationRate.d_yaw)
};
```

Keep `PrepareEkfTiming()` based only on payload timestamps. Update Doxygen text and filter locals so state semantics are easting, northing, heading, speed, yaw rate, and longitudinal acceleration.

- [ ] **Step 4: Run IMU manager and filter tests**

Expected: timing is independent of replay execution speed, yaw rate affects heading, and state/covariance remain finite for representative inputs.

---

### Task 5: Simulation Entry Point and Production Cleanup

**Files:**
- Create: `src/main2.cpp`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: `build/imu_data_good.db`, `config.yaml`, and `test/WMM.COF`.
- Produces: `build/imu_fusion_replay.csv` and process success/failure.

- [ ] **Step 1: Remove the hard-coded GPS reader from production startup**

Remove `GpsManager` construction, callback installation, start, and associated stop behavior from `main.cpp`. Retain production IMU/controller/database lifecycle behavior so the OFF executable no longer attempts `/dev/ttyTEST`.

- [ ] **Step 2: Implement read-only merged replay in `main2.cpp`**

Open the source database read-only and execute a `UNION ALL` query whose normalized columns represent GPS1 RMC, rotation vector, acceleration, and rotation rate events ordered by `host_timestamp_ns`, event priority, and source id.

Reconstruct existing payload structures, feed GPS through `GetGPSCallback()`, feed IMU through the retained non-owning manager pointer, and start the controller from the first valid RMC coordinate with a null serial reader.

- [ ] **Step 3: Implement CSV output**

Write the exact header fields from the design, use `std::setprecision(std::numeric_limits<double>::max_digits10)`, detect completed steps by comparing `databaseManager->GetStats().ekfQueued`, and write one row using `GetKFState()` and `GetKFCovariance()` whenever the counter increments.

- [ ] **Step 4: Build and run the simulation executable**

Run:

```bash
./build.sh s
./build/IMUPROC
```

Expected: replay reaches end of database, exits successfully, and creates `build/imu_fusion_replay.csv`.

---

### Task 6: Full Verification

**Files:**
- Verify all modified source, test, build, and generated CSV files.

**Interfaces:**
- Produces evidence that production and simulation configurations build and the complete replay is numerically usable.

- [ ] **Step 1: Verify simulation tests**

Run the simulation build and complete test suite. Expected: all discovered tests pass.

- [ ] **Step 2: Validate CSV structure and numerical output**

Check that the header has the timestamp, GPS, KF state, and 36 covariance columns; row count equals the synchronous EKF enqueue count; and all required KF numeric fields are finite.

- [ ] **Step 3: Verify input database preservation**

Record and compare the source database SHA-256 before and after replay. Expected: hashes are identical.

- [ ] **Step 4: Verify the production build**

Run:

```bash
./build.sh
```

Expected: `SIMULATION_MODE=OFF`, the build and tests pass, and the resulting executable contains no hard-coded `/dev/ttyTEST` startup path.

- [ ] **Step 5: Run formatting and consistency checks**

Run `cmake --build build`, `ctest --test-dir build --output-on-failure`, `bash -n build.sh`, and searches confirming no C++ `SIMULATION_MODE` symbol and no new private method declarations.
