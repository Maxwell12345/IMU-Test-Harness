#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>
#include <utility>

#include <boost/asio.hpp>

#include "utils.hpp"
#include "SerialComService.hpp"
#include "IMUSerialPortReader.hpp"
#include "BoostSerialPort.hpp"
#include "gps/GpsManager.hpp"
#include "DatabaseManager.hpp"
#include "RadarPositionNavigationController.hpp"
#include "YamlConfigService.hpp"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main(int argc,char** argv) {
    try {
        std::signal(SIGINT, signalHandler);

        YamlConfigService yamlConfigService("config.yaml");
        auto config = yamlConfigService.GetConfig();

        auto databaseManager = std::make_shared<DatabaseManager>("./IMUPROC_tests.db");
        auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(config.imuSerialPort,
                                                                         std::make_unique<BoostSerialPort>());
        auto imuManager = std::make_unique<IMUManager>(databaseManager);
        auto gpsManager = std::make_unique<GpsManagerBase>();

        RadarPositionNavigationController radarPositionNavigationController(config.kalmanValues,
                                                                            databaseManager,
                                                                            std::move(imuSerialPortReader),
                                                                            std::move(gpsManager),
                                                                            std::move(imuManager));

        radarPositionNavigationController.StartAndConfigureRadarPNT(0, 0);
        
        std::cout << "CTRL + C to terminate..." << std::endl;
        while(keepRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        radarPositionNavigationController.StopRadarPNT();
        radarPositionNavigationController.TotalDestruction();

        return EXIT_SUCCESS;
    } catch(const std::invalid_argument &e) {
        //TODO: LOG_ERROR HERE
        std::cout << "[ERROR]" << e.what() << std::endl;
    } catch (const std::runtime_error &e) {
        //TODO: LOG_ERROR HERE
        std::cout << "[ERROR]" << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
