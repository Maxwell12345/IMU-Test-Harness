// #include <gtest/gtest.h>
// #include <chrono>
// #include <atomic>
// #include <thread>
// #include <limits>

// #include <boost/make_shared.hpp>

// #include "RadarPositionNavigationController.hpp"
// #include "IMUManager.hpp"

// namespace {
//     std::atomic<int> sh2OpenCalls{0};
//     std::atomic<int> sh2CloseCalls{0};
//     std::atomic<int> sh2SetSensorCallbackCalls{0};
//     std::atomic<int> sh2SetSensorConfigCalls{0};
//     std::atomic<int> sh2ServiceCalls{0};
//     std::atomic<int> bno085HalCreateCalls{0};
//     std::atomic<int> forceOpenFailure{0};
// }
// namespace {
//     std::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>("./IMUPROC_tests.db");
// }

// sh2_Hal_t bno085_hal_create() {
//     bno085HalCreateCalls++;
//     sh2_Hal_t hal{};
//     return hal;
// }

// extern "C" {

// int sh2_open(sh2_Hal_t* hal, sh2_EventCallback_t* eventCallback, void* eventCookie) {
//     (void)hal;
//     (void)eventCallback;
//     (void)eventCookie;

//     sh2OpenCalls++;

//     if (forceOpenFailure.load()) {
//         return SH2_ERR;
//     }

//     return SH2_OK;
// }

// void sh2_close(void) {
//     sh2CloseCalls++;
// }

// int sh2_setSensorCallback(sh2_SensorCallback_t* callback, void* cookie) {
//     (void)callback;
//     (void)cookie;
//     sh2SetSensorCallbackCalls++;
//     return SH2_OK;
// }

// int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t* config) {
//     (void)sensorId;
//     (void)config;
//     sh2SetSensorConfigCalls++;
//     return SH2_OK;
// }

// void sh2_service(void) {
//     sh2ServiceCalls++;
//     std::this_thread::sleep_for(std::chrono::milliseconds(1));
// }

// }

// TEST(RadarPositionNavigationControllerTest, GetGPSCallbackUpdatesLatestGps) {
//     IMUManager imuManager(db);
//     RadarPositionNavigationController t;

//     GpsUpdate gps{};
//     gps.receiveTime = std::chrono::steady_clock::now();
//     gps.timestamp = 12345;
//     gps.latitude = 47.319065;
//     gps.longitude = 5.06832;
//     gps.heading = 91.0;
//     gps.fixQuality = 1;
//     gps.numSatellites = 8;
//     gps.hdop = 0.9;
//     gps.gpsTimestampMs = std::numeric_limits<uint32_t>::max();
//     gps.valid = true;

//     std::function<void(const GpsUpdate&)> callback = t.GetGPSCallback();

//     callback(gps);

//     std::optional<GpsUpdate> latest = imuManager.GetLatestGps();

//     ASSERT_TRUE(latest.has_value());
//     ASSERT_NEAR(latest->latitude, 47.319065, 1e-12);
//     ASSERT_NEAR(latest->longitude, 5.06832, 1e-12);
//     ASSERT_TRUE(latest->heading.has_value());
//     ASSERT_NEAR(latest->heading.value(), 91.0, 1e-12);
//     ASSERT_EQ(latest->fixQuality, 1);
//     ASSERT_EQ(latest->numSatellites, 8);
//     ASSERT_NEAR(latest->hdop, 0.9, 1e-12);
//     ASSERT_EQ(latest->gpsTimestampMs, std::numeric_limits<uint32_t>::max());
//     ASSERT_TRUE(latest->valid);
// }

// TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTConfiguresKFAndStartsReader) {
//     sh2OpenCalls = 0;
//     sh2CloseCalls = 0;
//     sh2SetSensorCallbackCalls = 0;
//     sh2SetSensorConfigCalls = 0;
//     sh2ServiceCalls = 0;
//     bno085HalCreateCalls = 0;
//     forceOpenFailure = 0;

//     RadarPositionNavigationController t;

//     t.StartAndConfigureRadarPNT(47.319065, 5.06832);

//     std::this_thread::sleep_for(std::chrono::milliseconds(5));

//     ASSERT_TRUE(t.m_isKFConfigured.load());
//     ASSERT_TRUE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_TRUE(t.m_sh2IsOpen.load());

//     ASSERT_EQ(bno085HalCreateCalls.load(), 1);
//     ASSERT_EQ(sh2OpenCalls.load(), 1);
//     ASSERT_EQ(sh2SetSensorCallbackCalls.load(), 1);
//     ASSERT_EQ(sh2SetSensorConfigCalls.load(), 2);
//     ASSERT_GT(sh2ServiceCalls.load(), 0);

//     ASSERT_NEAR(t.m_latestX(0), 5.06832, 1e-12);
//     ASSERT_NEAR(t.m_latestX(1), 47.319065, 1e-12);
//     ASSERT_NEAR(t.m_latestX(2), 1e-15, 1e-20);
//     ASSERT_NEAR(t.m_latestX(3), 1e-15, 1e-20);
//     ASSERT_NEAR(t.m_latestX(4), 1e-16, 1e-21);
//     ASSERT_NEAR(t.m_latestX(5), 1e-16, 1e-21);

//     t.StopRadarPNT();

//     ASSERT_FALSE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_FALSE(t.m_sh2IsOpen.load());
//     ASSERT_EQ(sh2CloseCalls.load(), 1);
// }

// TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTDoesNotStartReaderWhenOpenFails) {
//     sh2OpenCalls = 0;
//     sh2CloseCalls = 0;
//     sh2SetSensorCallbackCalls = 0;
//     sh2SetSensorConfigCalls = 0;
//     sh2ServiceCalls = 0;
//     bno085HalCreateCalls = 0;
//     forceOpenFailure = 1;

//     RadarPositionNavigationController t;

//     t.StartAndConfigureRadarPNT(47.319065, 5.06832);

//     std::this_thread::sleep_for(std::chrono::milliseconds(5));

//     ASSERT_TRUE(t.m_isKFConfigured.load());
//     ASSERT_FALSE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_FALSE(t.m_sh2IsOpen.load());

//     ASSERT_EQ(bno085HalCreateCalls.load(), 1);
//     ASSERT_EQ(sh2OpenCalls.load(), 1);
//     ASSERT_EQ(sh2SetSensorCallbackCalls.load(), 0);
//     ASSERT_EQ(sh2SetSensorConfigCalls.load(), 0);
//     ASSERT_EQ(sh2ServiceCalls.load(), 0);
//     ASSERT_EQ(sh2CloseCalls.load(), 0);

//     forceOpenFailure = 0;
// }

// TEST(RadarPositionNavigationControllerTest, StopRadarPNTStopsThreadAndClosesSh2Once) {
//     sh2OpenCalls = 0;
//     sh2CloseCalls = 0;
//     sh2SetSensorCallbackCalls = 0;
//     sh2SetSensorConfigCalls = 0;
//     sh2ServiceCalls = 0;
//     bno085HalCreateCalls = 0;
//     forceOpenFailure = 0;

//     RadarPositionNavigationController t;

//     t.StartIMUReader();

//     std::this_thread::sleep_for(std::chrono::milliseconds(5));

//     ASSERT_TRUE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_TRUE(t.m_sh2IsOpen.load());
//     ASSERT_TRUE(t.m_serviceThread.joinable());

//     t.StopRadarPNT();

//     ASSERT_FALSE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_FALSE(t.m_sh2IsOpen.load());
//     ASSERT_FALSE(t.m_serviceThread.joinable());
//     ASSERT_EQ(sh2CloseCalls.load(), 1);

//     t.StopRadarPNT();

//     ASSERT_FALSE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_FALSE(t.m_sh2IsOpen.load());
//     ASSERT_FALSE(t.m_serviceThread.joinable());
//     ASSERT_EQ(sh2CloseCalls.load(), 1);
// }

// TEST(RadarPositionNavigationControllerTest, TotalDestructionStopsReaderCleansKFAndZerosLatestState) {
//     sh2OpenCalls = 0;
//     sh2CloseCalls = 0;
//     sh2SetSensorCallbackCalls = 0;
//     sh2SetSensorConfigCalls = 0;
//     sh2ServiceCalls = 0;
//     bno085HalCreateCalls = 0;
//     forceOpenFailure = 0;

//     RadarPositionNavigationController t;

//     t.StartAndConfigureRadarPNT(47.319065, 5.06832);

//     std::this_thread::sleep_for(std::chrono::milliseconds(5));

//     ASSERT_TRUE(t.m_isKFConfigured.load());
//     ASSERT_TRUE(t.m_sh2IsOpen.load());
//     ASSERT_FALSE(t.m_latestX.isZero(0.0));

//     t.TotalDestruction();

//     ASSERT_FALSE(t.m_isKFConfigured.load());
//     ASSERT_FALSE(t.m_sh2ServiceIsRunning.load());
//     ASSERT_FALSE(t.m_sh2IsOpen.load());
//     ASSERT_TRUE( t.m_latestX.isZero(0.0));
//     ASSERT_TRUE(t.m_latestP.isZero(0.0));
//     ASSERT_EQ(sh2CloseCalls.load(), 1);
// }

// TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterSetsInitialStateAndCovariance) {
//     RadarPositionNavigationController t;

//     t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);

//     ASSERT_NEAR(t.m_latestX(0), 5.06832, 1e-12);
//     ASSERT_NEAR(t.m_latestX(1), 47.319065, 1e-12);
//     ASSERT_NEAR(t.m_latestX(2), 1e-15, 1e-20);
//     ASSERT_NEAR(t.m_latestX(3), 1e-15, 1e-20);
//     ASSERT_NEAR(t.m_latestX(4), 1e-16, 1e-21);
//     ASSERT_NEAR(t.m_latestX(5), 1e-16, 1e-21);

//     ASSERT_NEAR(t.m_latestP(0, 0), 1e-10,1e-20);
//     ASSERT_NEAR(t.m_latestP(1, 1), 1e-10, 1e-20);
//     ASSERT_NEAR(t.m_latestP(2, 2), 1e-8, 1e-18);
//     ASSERT_NEAR(t.m_latestP(3, 3), 1e-8, 1e-18);
//     ASSERT_NEAR(t.m_latestP(4, 4), 1e-10, 1e-20);
//     ASSERT_NEAR(t.m_latestP(5, 5), 1e-10, 1e-20);

//     for (int i = 0; i < 6; i++) {
//         for (int j = 0; j < 6; j++) {
//             if (i != j) {
//                 ASSERT_NEAR(t.m_latestP(i, j), 0.0, 1e-30) << "Index i: " + std::to_string(i) + " and j: " + std::to_string(j);
//             }
//         }
//     }
// }

// TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsInvalidPercentiles) {
//     RadarPositionNavigationController t;

//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.0, 0.95, 0.20, 0.95), std::runtime_error);
//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 1.0, 0.95, 0.20, 0.95), std::runtime_error);
//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.0, 0.95), std::runtime_error);
//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 1.0, 0.95), std::runtime_error);
// }

// TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsLowerPercentileGreaterThanUpperPercentile) {
//     RadarPositionNavigationController t;

//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.95, 0.20, 0.20, 0.95), std::runtime_error);
//     ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.95, 0.20), std::runtime_error);
// }

// TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyReturnsWithoutConfiguredKF) {
//     RadarPositionNavigationController t;

//     Vector6d imuVec;
//     imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

//     t.KFCallbackImuOnly(0.01, imuVec);

//     ASSERT_TRUE(t.m_latestX.isZero(0.0));
//     ASSERT_TRUE(t.m_latestP.isZero(0.0));
// }

// TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsReturnsWithoutConfiguredKF) {
//     RadarPositionNavigationController t;

//     Vector6d imuVec;
//     imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

//     Vector6d gpsVec;
//     gpsVec << 5.06832, 47.319065, 0.0, 0.0, 0.0, 0.0;

//     t.KFCallbackWithGps(0.01, imuVec, gpsVec);

//     ASSERT_TRUE(t.m_latestX.isZero(0.0));
//     ASSERT_TRUE(t.m_latestP.isZero(0.0));
// }

// TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyProducesNonFiniteStateBecauseKFUsesSingularR) {
//     RadarPositionNavigationController t;

//     t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);
//     t.m_isKFConfigured.store(true);

//     Vector6d imuVec;
//     imuVec << 0.0, 0.0, 1e-9, -1e-9, 1e-10, -1e-10;

//     t.KFCallbackImuOnly(0.01, imuVec);

//     ASSERT_TRUE(t.m_isKFConfigured.load());
//     ASSERT_FALSE(std::isfinite(t.m_latestX(0)));
// }

// TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsProducesNonFiniteStateBecauseKFUsesSingularR) {
//     RadarPositionNavigationController t;

//     t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);
//     t.m_isKFConfigured.store(true);

//     Vector6d gpsVec;
//     gpsVec << 5.0683200001, 47.3190650001, 0.0, 0.0, 0.0, 0.0;

//     Vector6d imuVec;
//     imuVec << 0.0, 0.0, 1e-9, -1e-9, 1e-10, -1e-10;

//     t.KFCallbackWithGps(0.01, imuVec, gpsVec);

//     ASSERT_TRUE(t.m_isKFConfigured.load());
//     ASSERT_FALSE(std::isfinite(t.m_latestX(0)));
// }