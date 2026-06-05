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

#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

extern "C" {
#include "sh2_SensorValue.h"
}

#include "imu_data.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

class DatabaseManager {
    
};

static std::atomic<bool> g_running{true};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;
    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        std::cerr << "[WARN] Failed to enable sensor id=" << sensor_id << "\n";
    }
}


// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

int main() {
    boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>();

    IMUGPSFusionKF_2D_ConstantAcceleration ekf;

    IMUManager::Initialize(
    db,
    [&ekf](double dt, Vector6d& z_IMU) {
        return ekf.Step(dt, z_IMU);
    },
    [&ekf](double dt, Vector6d& z_GPS, Vector6d& z_IMU) {
        return ekf.Step(dt, z_GPS, z_IMU);
    });

    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        return EXIT_FAILURE;
    }

    sh2_setSensorCallback(IMUManager::SensorCallback, nullptr);

    // enable_sensor(SH2_ACCELEROMETER,             2'500);
    enable_sensor(SH2_LINEAR_ACCELERATION,       2'500);
    // enable_sensor(SH2_GYROSCOPE_CALIBRATED,      2'500);
    // enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10'000);
    enable_sensor(SH2_ROTATION_VECTOR,           2'500);
    // enable_sensor(SH2_GAME_ROTATION_VECTOR,      2'500);

    std::thread service_thread([]() {
        while (g_running) {
            sh2_service();
        }
    });

    int PRINT_INTERVAL = 1000;

    std::cout << "BNO085 streaming — press Esc to stop\n\n";

    while (g_running) {
        char ch = 0;
        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1b') {
            g_running = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    service_thread.join();
    sh2_close();
    std::cout << "\nShutdown complete.\n";
}
