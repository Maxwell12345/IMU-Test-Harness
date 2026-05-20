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

// ---------------------------------------------------------------------------
// Shared IMU snapshot — updated by sensor_callback, read by the print loop
// ---------------------------------------------------------------------------

struct Snapshot {
    std::mutex             mtx;
    AccelerometerData      accel{};
    LinearAccelData        linear_accel{};
    GyroscopeData          gyro{};
    MagnetometerData       mag{};
    RotationVectorData     rot{};
    GameRotationVectorData game_rot{};
};

static Snapshot        g_snapshot;
static std::atomic<bool> g_running{true};

// ---------------------------------------------------------------------------
// Sensor callback — fires on the service thread for every decoded report
// ---------------------------------------------------------------------------

static void sensor_callback(void* /*cookie*/, sh2_SensorEvent_t* event) {
    sh2_SensorValue_t val{};
    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) return;

    std::lock_guard<std::mutex> lock(g_snapshot.mtx);

    switch (val.sensorId) {
        case SH2_ACCELEROMETER:
            g_snapshot.accel.timestamp_us = val.timestamp;
            g_snapshot.accel.x            = val.un.accelerometer.x;
            g_snapshot.accel.y            = val.un.accelerometer.y;
            g_snapshot.accel.z            = val.un.accelerometer.z;
            g_snapshot.accel.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_LINEAR_ACCELERATION:
            g_snapshot.linear_accel.timestamp_us = val.timestamp;
            g_snapshot.linear_accel.x            = val.un.linearAcceleration.x;
            g_snapshot.linear_accel.y            = val.un.linearAcceleration.y;
            g_snapshot.linear_accel.z            = val.un.linearAcceleration.z;
            g_snapshot.linear_accel.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            g_snapshot.gyro.timestamp_us = val.timestamp;
            g_snapshot.gyro.x            = val.un.gyroscope.x;
            g_snapshot.gyro.y            = val.un.gyroscope.y;
            g_snapshot.gyro.z            = val.un.gyroscope.z;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            g_snapshot.mag.timestamp_us = val.timestamp;
            g_snapshot.mag.x            = val.un.magneticField.x;
            g_snapshot.mag.y            = val.un.magneticField.y;
            g_snapshot.mag.z            = val.un.magneticField.z;
            g_snapshot.mag.accuracy     = static_cast<uint8_t>(val.status & 0x03);
            break;

        case SH2_ROTATION_VECTOR:
            g_snapshot.rot.timestamp_us = val.timestamp;
            g_snapshot.rot.i            = val.un.rotationVector.i;
            g_snapshot.rot.j            = val.un.rotationVector.j;
            g_snapshot.rot.k            = val.un.rotationVector.k;
            g_snapshot.rot.real         = val.un.rotationVector.real;
            g_snapshot.rot.accuracy     = val.un.rotationVector.accuracy;
            break;

        case SH2_GAME_ROTATION_VECTOR:
            g_snapshot.game_rot.timestamp_us = val.timestamp;
            g_snapshot.game_rot.i            = val.un.gameRotationVector.i;
            g_snapshot.game_rot.j            = val.un.gameRotationVector.j;
            g_snapshot.game_rot.k            = val.un.gameRotationVector.k;
            g_snapshot.game_rot.real         = val.un.gameRotationVector.real;
            break;

        default:
            break;
    }
}

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

static void bodyToGlobal(float qx, float qy, float qz, float qw,
                         float body[3], float global[3]) {
    float dcm[3][3] = {
        {1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw},
        {2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw},
        {2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy}
    };

    global[0] = dcm[0][0]*body[0] + dcm[0][1]*body[1] + dcm[0][2]*body[2];
    global[1] = dcm[1][0]*body[0] + dcm[1][1]*body[1] + dcm[1][2]*body[2];
    global[2] = dcm[2][0]*body[0] + dcm[2][1]*body[1] + dcm[2][2]*body[2];
}

static void print_snapshot() {
    Snapshot snap_copy;
    float body[3];
    float global[3];
    {
        std::lock_guard<std::mutex> lock(g_snapshot.mtx);
        snap_copy.accel       = g_snapshot.accel;
        snap_copy.linear_accel = g_snapshot.linear_accel;
        snap_copy.gyro        = g_snapshot.gyro;
        snap_copy.mag         = g_snapshot.mag;
        snap_copy.rot         = g_snapshot.rot;
        snap_copy.game_rot    = g_snapshot.game_rot;
    }
    body[0] = snap_copy.linear_accel.x;
    body[1] = snap_copy.linear_accel.y;
    body[2] = snap_copy.linear_accel.z;
    bodyToGlobal(snap_copy.rot.i, snap_copy.rot.j, snap_copy.rot.k, snap_copy.rot.real, body, global);

    std::cout << "----------------------------------------\n"
              << snap_copy.accel        << "\n"
              << snap_copy.linear_accel << "\n"
              << snap_copy.gyro         << "\n"
              << snap_copy.mag          << "\n"
              << snap_copy.rot          << "\n"
              << snap_copy.game_rot     << "\n"
              << global[0] << " | " << global[1] << " | " << global[2] << "\n";
}


// RAII wrapper: puts stdin into raw non-blocking mode, restores on destruction
struct RawTerminal {
    struct termios saved{};

    RawTerminal() {
        tcgetattr(STDIN_FILENO, &saved);
        struct termios t = saved;
        t.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        t.c_cc[VMIN]  = 0;  // non-blocking: return immediately
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    ~RawTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    }
};


// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

void run_demo2() {
    RawTerminal term;

    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        return;
    }

    sh2_setSensorCallback(sensor_callback, nullptr);

    enable_sensor(SH2_ACCELEROMETER,             2'500);
    enable_sensor(SH2_LINEAR_ACCELERATION,       2'500);
    enable_sensor(SH2_GYROSCOPE_CALIBRATED,      2'500);
    enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10'000);
    enable_sensor(SH2_ROTATION_VECTOR,           2'500);
    enable_sensor(SH2_GAME_ROTATION_VECTOR,      2'500);

    std::thread service_thread([]() {
        while (g_running) {
            sh2_service();
        }
    });

    int PRINT_INTERVAL = 1000;

    std::cout << "BNO085 streaming — press Esc to stop\n\n";

    using clock = std::chrono::steady_clock;
    auto next_print = clock::now() + std::chrono::milliseconds(PRINT_INTERVAL);

    while (g_running) {
        char ch = 0;
        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1b') {
            g_running = false;
            break;
        }

        auto now = clock::now();
        if (now >= next_print) {
            print_snapshot();
            next_print = now + std::chrono::milliseconds(PRINT_INTERVAL);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    service_thread.join();
    sh2_close();
    std::cout << "\nShutdown complete.\n";
}
