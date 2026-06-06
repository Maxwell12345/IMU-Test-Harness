#include "IMUManager.hpp"
#include "IMUGPSFusionKF.hpp"

#include "demo.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fstream>

#include "bno085_hal.hpp"

extern "C" {
#include "sh2_SensorValue.h"
#include "sh2.h"
}

#include "imu_data.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

class DatabaseManager {
    
};

static std::atomic<bool> g_running{true};

struct SensorRow {
    double epoch_s;
    long long sensor_timestamp_us;
    int sensor_id;
    std::string sensor_name;
    int status, accuracy;
    std::optional<double> x, y, z;
    std::optional<double> i, j, k, real;
    std::optional<double> rotation_accuracy;
};

struct GpsRow {
    double epoch_s;
    std::string sentence_id;
    std::optional<double> lat, lon;
};

void EnableSensor(sh2_SensorId_t sensor_id, uint32_t interval_us);
size_t ReadSensorCsv(const std::string& path, std::vector<SensorRow> &rows);
size_t ReadGpsCsv(const std::string& path, std::vector<GpsRow> &rows);

int main() {
    boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>();

    IMUGPSFusionKF_2D_ConstantAcceleration ekf;
    auto ekfNoGps = [&ekf](double dt, Vector6d& z_IMU) {
                        return ekf.Step(dt, z_IMU);
                    };
    auto ekfWithGps = [&ekf](double dt, Vector6d& z_GPS, Vector6d& z_IMU) {
                            return ekf.Step(dt, z_GPS, z_IMU);
                        };

    IMUManager::Initialize(db,
                        ekfNoGps,
                        ekfWithGps);

    // sh2_Hal_t hal = bno085_hal_create();
    // if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
    //     std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
    //     return EXIT_FAILURE;
    // }

    // sh2_setSensorCallback(IMUManager::SensorCallback, nullptr);

    // EnableSensor(SH2_LINEAR_ACCELERATION,       2'500);
    // EnableSensor(SH2_ROTATION_VECTOR,           2'500);

    // std::thread service_thread([]() {
    //     while (g_running) {
    //         sh2_service();
    //     }
    // });
    // std::cout << "BNO085 streaming — press Esc to stop\n\n";

    std::cout << "Reading csv files..." << std::endl;

    std::vector<SensorRow> sensorRows;
    size_t sensorRowSize = ReadSensorCsv("./logs/imu_measurements.csv", sensorRows);
    std::vector<GpsRow> gpsRows;
    size_t gpsRowsSize = ReadGpsCsv("./logs/nmea_sentences.csv", gpsRows);

    std::cout << sensorRowSize << " Sensor rows" << std::endl
              << gpsRowsSize << " Nmea rows" << std::endl;

    std::thread service_thread([&sensorRows, &gpsRows]() {
        size_t sensorIdx = 0;
        size_t gpsIdx = 0;

        while(gpsRows[gpsIdx].epoch_s < 1779986868) {
            gpsIdx++;
        }

        while(sensorRows[sensorIdx].epoch_s < 1779986868) {
            sensorIdx++;
        }

        while (g_running) {

            if(sensorRows[sensorIdx].epoch_s < gpsRows[gpsIdx].epoch_s) {
                auto& row = sensorRows[sensorIdx];

                sh2_SensorValue value;
                value.sensorId = row.sensor_id;
                value.timestamp = row.sensor_timestamp_us;

                switch (value.sensorId) {
                    case SH2_LINEAR_ACCELERATION:
                        value.un.linearAcceleration.x = row.x.value();
                        value.un.linearAcceleration.y = row.y.value();
                        value.un.linearAcceleration.z = row.z.value();
                        break;

                    case SH2_ROTATION_VECTOR:
                        value.un.rotationVector.i = row.i.value();
                        value.un.rotationVector.j = row.j.value();
                        value.un.rotationVector.k = row.k.value();
                        value.un.rotationVector.real = row.real.value();
                        break;
                }

                IMUManager::SensorCallback(value);

                if(sensorIdx < sensorRows.size() - 1) {
                    ++sensorIdx;
                }
            } else {
                auto& row = gpsRows[gpsIdx];

                if(row.lat.has_value() && row.lon.has_value()) {
                    GpsUpdate gpsUpdate;
                    gpsUpdate.latitude = row.lat.value();
                    gpsUpdate.longitude = row.lon.value();
                    gpsUpdate.valid = true;
                    gpsUpdate.receiveTime = std::chrono::steady_clock::now();
                    gpsUpdate.gpsTimestampMs = static_cast<uint32_t>(row.epoch_s);
                    IMUManager::UpdateLatestGps(gpsUpdate);
                }

                if(gpsIdx < gpsRows.size() - 1) {
                    ++gpsIdx;
                }
            }

            if ((gpsIdx == (gpsRows.size() - 1)) && (sensorIdx == (sensorRows.size() - 1))) {
                g_running = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::cout << "BNO085 streaming — press Esc to stop\n\n";

    while (g_running) {
        char ch = 0;
        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == 'C') {
            g_running = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    service_thread.join();
    // sh2_close();
    std::cout << "\nShutdown complete.\n";
}

void EnableSensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;
    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        std::cerr << "[WARN] Failed to enable sensor id=" << sensor_id << "\n";
    }
}

static std::optional<double> ParseOpt(const std::string& s) {
    return s.empty() ? std::nullopt : std::optional<double>(std::stod(s));
}

size_t ReadSensorCsv(const std::string& path, std::vector<SensorRow>& rows) {
    std::ifstream f(path);
    std::string line;
    std::getline(f, line);

    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::vector<std::string> cols;
        std::string tok;
        while (std::getline(ss, tok, ','))
            cols.push_back(tok);
        if (cols.size() > 14)
            continue;

        cols.resize(14);

        auto& name = cols[3];
        if (name.size() >= 2 && name.front() == '"') name = name.substr(1, name.size() - 2);

        rows.push_back({
            std::stod(cols[0]),
            std::stoll(cols[1]),
            std::stoi(cols[2]),
            name,
            std::stoi(cols[4]),
            std::stoi(cols[5]),
            ParseOpt(cols[6]),  ParseOpt(cols[7]),  ParseOpt(cols[8]),
            ParseOpt(cols[9]),  ParseOpt(cols[10]), ParseOpt(cols[11]),
            ParseOpt(cols[12]), ParseOpt(cols[13])
        });
    }
    return rows.size();
}

static std::optional<double> ParseDm(const std::string& val, const std::string& hemi) {
    if (val.empty()) return std::nullopt;
    double raw = std::stod(val);
    int deg = static_cast<int>(raw / 100);
    double minutes = raw - deg * 100.0;
    double dd = deg + minutes / 60.0;
    if (hemi == "S" || hemi == "W") dd = -dd;
    return dd;
}

static std::vector<std::string> Split(const std::string& s, char delim = ',') {
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, delim)) out.push_back(tok);
    return out;
}

size_t ReadGpsCsv(const std::string& path, std::vector<GpsRow>& rows) {
    std::ifstream f(path);
    std::string line;
    std::getline(f, line);

    while (std::getline(f, line)) {
        auto cols = Split(line);
        if (cols.size() < 6) continue;

        std::string id = cols[3];
        auto strip = [](std::string& s) {
            if (s.size() >= 2 && s.front() == '"') s = s.substr(1, s.size() - 2);
        };
        strip(id);
        if (id != "GPGGA" && id != "GPRMC" && id != "GPGLL") continue;

        std::string nmea = cols[5];
        for (size_t i = 6; i < cols.size(); ++i) nmea += ',' + cols[i];
        strip(nmea);

        auto f2 = Split(nmea);

        std::optional<double> lat, lon;

        if (id == "GPGGA" && f2.size() > 5) {
            lat = ParseDm(f2[2], f2[3]);
            lon = ParseDm(f2[4], f2[5]);
        } else if (id == "GPRMC" && f2.size() > 6) {
            lat = ParseDm(f2[3], f2[4]);
            lon = ParseDm(f2[5], f2[6]);
        } else if (id == "GPGLL" && f2.size() > 4) {
            lat = ParseDm(f2[1], f2[2]);
            lon = ParseDm(f2[3], f2[4]);
        }

        rows.push_back({std::stod(cols[0]), id, lat, lon});
    }
    return rows.size();
}