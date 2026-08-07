/******************************************************************************
 * File:             main2.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Replays recorded GPS and IMU payloads from SQLite through
 *                   IMUManager and RadarPositionNavigationController, writing
 *                   synchronized ENU Kalman-filter results to CSV.
 *
 ******************************************************************************/

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <SQLiteCpp/SQLiteCpp.h>

#include "DatabaseManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUManager.hpp"
#include "RadarPositionNavigationController.hpp"
#include "YamlConfigService.hpp"
#include "gps/GpsManager.hpp"
#include "gps/NmeaReader.hpp"
#include "imu_data.hpp"

namespace {

constexpr const char* INPUT_DATABASE_PATH = "./imu_data_good.db";
constexpr const char* OUTPUT_CSV_PATH = "./output.csv";
constexpr const char* CONFIG_PATH = "config.yaml";
constexpr const char* MAGNETIC_MODEL_PATH = "./WMM.COF";

enum class ReplayEventType : int {
    Gps = 0,
    RotationRate = 1,
    RotationVector = 2,
    LinearAcceleration = 3
};

GpsUpdate ReadInitialGps(SQLite::Database& inputDatabase, uint64_t& gpsTimestampNs) {
    SQLite::Statement initialGpsQuery(
        inputDatabase,
        R"sql(
            SELECT host_timestamp_ns, nmea
            FROM gps1_nmea
            WHERE nmea LIKE '$G_RMC,%'
            ORDER BY host_timestamp_ns, id
        )sql");

    while (initialGpsQuery.executeStep()) {
        const uint64_t timestampNs = static_cast<uint64_t>(initialGpsQuery.getColumn(0).getInt64());
        const std::string sentence = initialGpsQuery.getColumn(1).getString();
        const NmeaMessage message = NmeaReader::Parse(sentence);
        const GpsUpdate update = GpsManager::BuildGpsUpdate(message, timestampNs);

        if (update.valid && update.latitude != 0.0 && update.longitude != 0.0) {
            gpsTimestampNs = timestampNs;
            return update;
        }
    }

    throw std::runtime_error("No valid GPS1 RMC record is available to establish the ENU origin");
}

void WriteCsvHeader(std::ofstream& csvOutput) {
    csvOutput << "measurement_host_timestamp_ns"
              << ",gps_host_timestamp_ns"
              << ",gps_latitude_deg"
              << ",gps_longitude_deg"
              << ",gps_course_deg"
              << ",kf_easting_m"
              << ",kf_northing_m"
              << ",kf_speed_mps"
              << ",kf_heading_rad"
              << ",kf_heading_deg";

    for (Eigen::Index row = 0; row < 6; ++row) {
        for (Eigen::Index column = 0; column < 6; ++column) {
            csvOutput << ",kf_covariance_" << row << '_' << column;
        }
    }

    csvOutput << '\n';
}

void WriteCsvRow(std::ofstream& csvOutput,
                 uint64_t measurementTimestampNs,
                 uint64_t gpsTimestampNs,
                 const GpsUpdate& gpsUpdate,
                 const Vector6d& state,
                 const Matrix6d& covariance) {
    constexpr double pi = 3.141592653589793238462643383279502884;
    constexpr double radiansToDegrees = 180.0 / pi;
    constexpr double fullTurnRadians = 2.0 * pi;

    if (!state.allFinite() || !covariance.allFinite()) {
        throw std::runtime_error("Kalman-filter output contains a non-finite value");
    }

    csvOutput << measurementTimestampNs
              << ',' << gpsTimestampNs
              << ',' << gpsUpdate.latitude
              << ',' << gpsUpdate.longitude
              << ',';

    if (gpsUpdate.heading.has_value()) {
        csvOutput << *gpsUpdate.heading;
    }

    double normalizedHeadingRadians = std::fmod(state(2), fullTurnRadians);
    if (normalizedHeadingRadians < 0.0) {
        normalizedHeadingRadians += fullTurnRadians;
    }

    csvOutput << ',' << state(0)
              << ',' << state(1)
              << ',' << state(3)
              << ',' << normalizedHeadingRadians
              << ',' << normalizedHeadingRadians * radiansToDegrees;

    for (Eigen::Index row = 0; row < covariance.rows(); ++row) {
        for (Eigen::Index column = 0; column < covariance.cols(); ++column) {
            csvOutput << ',' << covariance(row, column);
        }
    }

    csvOutput << '\n';
}

} // namespace

int main() {
    try {
        if (!std::filesystem::is_regular_file(INPUT_DATABASE_PATH)) {
            throw std::runtime_error("Recorded input database does not exist");
        }

        SQLite::Database inputDatabase(INPUT_DATABASE_PATH, SQLite::OPEN_READONLY);

        uint64_t latestGpsTimestampNs = 0;
        GpsUpdate latestGps = ReadInitialGps(inputDatabase, latestGpsTimestampNs);

        YamlConfigService configService(CONFIG_PATH);
        const YamlConfig config = configService.GetConfig();

        auto databaseManager = std::make_shared<DatabaseManager>(":memory:");
        databaseManager->Start();

        auto imuManager = std::make_unique<IMUManager>(databaseManager, MAGNETIC_MODEL_PATH);
        IMUManager* const imuManagerReplayInput = imuManager.get();

        RadarPositionNavigationController controller(config.kalmanValues,
                                                      databaseManager,
                                                      nullptr,
                                                      std::move(imuManager));
        controller.StartAndConfigureRadarPNT(latestGps.latitude, latestGps.longitude);
        const auto gpsCallback = controller.GetGPSCallback();
        gpsCallback(latestGps);

        std::ofstream csvOutput(OUTPUT_CSV_PATH, std::ios::out | std::ios::trunc);
        if (!csvOutput.is_open()) {
            throw std::runtime_error("Unable to create simulation CSV output");
        }
        csvOutput << std::setprecision(std::numeric_limits<double>::max_digits10);
        WriteCsvHeader(csvOutput);

        SQLite::Statement eventQuery(
            inputDatabase,
            R"sql(
                SELECT event_type,
                       host_timestamp_ns,
                       source_id,
                       sensor_timestamp_us,
                       value_1,
                       value_2,
                       value_3,
                       value_4,
                       value_5,
                       nmea
                FROM (
                    SELECT 0 AS event_type,
                           host_timestamp_ns,
                           id AS source_id,
                           0 AS sensor_timestamp_us,
                           0.0 AS value_1,
                           0.0 AS value_2,
                           0.0 AS value_3,
                           0.0 AS value_4,
                           0.0 AS value_5,
                           nmea
                    FROM gps1_nmea
                    WHERE nmea LIKE '$G_RMC,%'

                    UNION ALL

                    SELECT 1,
                           host_timestamp_ns,
                           id,
                           timestamp_us,
                           d_roll,
                           d_pitch,
                           d_yaw,
                           0.0,
                           0.0,
                           NULL
                    FROM imu_delta_rotation_rate

                    UNION ALL

                    SELECT 2,
                           host_timestamp_ns,
                           id,
                           timestamp_us,
                           i,
                           j,
                           k,
                           real,
                           accuracy,
                           NULL
                    FROM imu_rotation_vector

                    UNION ALL

                    SELECT 3,
                           host_timestamp_ns,
                           id,
                           timestamp_us,
                           ax,
                           ay,
                           az,
                           0.0,
                           0.0,
                           NULL
                    FROM imu_linear_acceleration
                )
                ORDER BY host_timestamp_ns, event_type, source_id
            )sql");

        uint64_t csvRowCount = 0;

        while (eventQuery.executeStep()) {
            const ReplayEventType eventType = static_cast<ReplayEventType>(eventQuery.getColumn(0).getInt());
            const uint64_t hostTimestampNs = static_cast<uint64_t>(eventQuery.getColumn(1).getInt64());

            // if (eventType == ReplayEventType::Gps) {
            //     const NmeaMessage message = NmeaReader::Parse(eventQuery.getColumn(9).getString());
            //     const GpsUpdate gpsUpdate = GpsManager::BuildGpsUpdate(message, hostTimestampNs);

            //     if (gpsUpdate.valid && gpsUpdate.latitude != 0.0 && gpsUpdate.longitude != 0.0) {
            //         latestGps = gpsUpdate;
            //         latestGpsTimestampNs = hostTimestampNs;
            //         gpsCallback(latestGps);
            //     }

            //     continue;
            // }

            const uint64_t sensorTimestampUs =
                static_cast<uint64_t>(eventQuery.getColumn(3).getInt64());
            const uint64_t ekfCountBefore = databaseManager->GetStats().ekfQueued.load();

            switch (eventType) {
                case ReplayEventType::RotationRate: {
                    const Raw_RotationRate rotationRate{
                        static_cast<float>(eventQuery.getColumn(4).getDouble()),
                        static_cast<float>(eventQuery.getColumn(5).getDouble()),
                        static_cast<float>(eventQuery.getColumn(6).getDouble()),
                        sensorTimestampUs
                    };
                    imuManagerReplayInput->SensorCallback(std::nullopt, std::nullopt, rotationRate);
                    break;
                }

                case ReplayEventType::RotationVector: {
                    const Raw_RotationVectorWAcc rotationVector{
                        static_cast<float>(eventQuery.getColumn(4).getDouble()),
                        static_cast<float>(eventQuery.getColumn(5).getDouble()),
                        static_cast<float>(eventQuery.getColumn(6).getDouble()),
                        static_cast<float>(eventQuery.getColumn(7).getDouble()),
                        static_cast<float>(eventQuery.getColumn(8).getDouble()),
                        sensorTimestampUs
                    };
                    imuManagerReplayInput->SensorCallback(rotationVector, std::nullopt, std::nullopt);
                    break;
                }

                case ReplayEventType::LinearAcceleration: {
                    const Raw_Accelerometer linearAcceleration{
                        static_cast<float>(eventQuery.getColumn(4).getDouble()),
                        static_cast<float>(eventQuery.getColumn(5).getDouble()),
                        static_cast<float>(eventQuery.getColumn(6).getDouble()),
                        sensorTimestampUs
                    };
                    imuManagerReplayInput->SensorCallback(std::nullopt, linearAcceleration, std::nullopt);
                    break;
                }

                case ReplayEventType::Gps:
                    break;
            }

            const uint64_t ekfCountAfter = databaseManager->GetStats().ekfQueued.load();
            if (ekfCountAfter > ekfCountBefore) {
                WriteCsvRow(csvOutput,
                            hostTimestampNs,
                            latestGpsTimestampNs,
                            latestGps,
                            controller.GetKFState(),
                            controller.GetKFCovariance());
                ++csvRowCount;
            }
        }

        if (csvRowCount == 0) {
            throw std::runtime_error("Simulation completed without a Kalman-filter update");
        }

        csvOutput.flush();
        if (!csvOutput.good()) {
            throw std::runtime_error("Simulation CSV output could not be flushed");
        }

        controller.StopRadarPNT();
        databaseManager->Stop();
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception) {
        std::cerr << "[ERROR] " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
