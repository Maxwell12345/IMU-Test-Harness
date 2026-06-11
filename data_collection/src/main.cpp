#include "BrokerService.hpp"
#include "GpsService.hpp"
#include "ImuService.hpp"
#include "Logger.hpp"
#include "SqliteManager.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void handleSignal(int) {
    g_running.store(false);
}

constexpr uint16_t kUbloxVendorId = 0x1546;
constexpr uint16_t kZedF9pProductId = 0x01a9;   // SparkFun GPS-RTK-SMA (ZED-F9P)
constexpr uint16_t kUblox7ProductId = 0x01a7;   // VFAN-USB UG-353 (u-blox 7)

constexpr auto kHeartbeatPeriod = std::chrono::seconds(60);

void applyLogLevelFromEnv() {
    const char* level = std::getenv("DATA_COLLECTION_LOG_LEVEL");
    if (level == nullptr) {
        return;
    }
    if (std::strcmp(level, "debug") == 0) {
        Logger::setLevel(LogLevel::Debug);
    } else if (std::strcmp(level, "info") == 0) {
        Logger::setLevel(LogLevel::Info);
    } else if (std::strcmp(level, "warn") == 0) {
        Logger::setLevel(LogLevel::Warn);
    } else if (std::strcmp(level, "error") == 0) {
        Logger::setLevel(LogLevel::Error);
    }
}

std::string databasePath() {
    const char* path = std::getenv("DATA_COLLECTION_DB_PATH");
    if (path != nullptr && path[0] != '\0') {
        return path;
    }
    return "imu_data.db";
}

}  // namespace

int main() {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    applyLogLevelFromEnv();
    LOG_INFO("main") << "IMU + GPS data collection harness starting";

    try {
        const std::string dbPath = databasePath();
        SqliteManager sqliteManager(dbPath);
        BrokerService broker(sqliteManager);

        ImuService imuService(broker);
        GpsService sparkfunGps(broker, "sparkfun-zed-f9p", kUbloxVendorId, kZedF9pProductId);
        GpsService vfanGps(broker, "vfan-ug353", kUbloxVendorId, kUblox7ProductId);

        broker.start();

        try {
            imuService.start();
        } catch (const std::exception& e) {
            LOG_ERROR("main") << "IMU failed to start, continuing without it: " << e.what();
        }

        sparkfunGps.start();
        vfanGps.start();

        LOG_INFO("main") << "Running — stop with Ctrl+C, 'systemctl stop', or kill "
                            "(SIGINT/SIGTERM)";

        const auto startedAt = std::chrono::steady_clock::now();
        auto lastHeartbeat = startedAt;

        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            const auto now = std::chrono::steady_clock::now();
            if (now - lastHeartbeat >= kHeartbeatPeriod) {
                const auto uptime =
                    std::chrono::duration_cast<std::chrono::seconds>(now - startedAt).count();
                LOG_INFO("main") << "Alive, uptime " << uptime << "s";
                lastHeartbeat = now;
            }
        }

        LOG_INFO("main") << "Shutdown signal received, stopping services";
        imuService.stop();
        sparkfunGps.stop();
        vfanGps.stop();
        broker.stop();
    } catch (const std::exception& e) {
        LOG_ERROR("main") << "Fatal: " << e.what();
        return 1;
    }

    LOG_INFO("main") << "Shutdown complete";
    return 0;
}
