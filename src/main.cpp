/******************************************************************************
 * File:             main.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Starts the production IMU navigation path using configured
 *                   serial hardware, persistence, and Kalman-filter services.
 *
 ******************************************************************************/

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
#include "DatabaseManager.hpp"
#include "RadarPositionNavigationController.hpp"
#include "YamlConfigService.hpp"

std::atomic<bool> keepRunning = true;

int main(int argc, char** argv) {
    try {
        YamlConfigService yamlConfigService("config.yaml");
        const auto config = yamlConfigService.GetConfig();

        auto databaseManager = std::make_shared<DatabaseManager>("./IMUPROC_tests.db");
        databaseManager->Start();
        auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(config.imuSerialPort,
                                                                         std::make_unique<BoostSerialPort>());
        auto imuManager = std::make_unique<IMUManager>(databaseManager, "build/WMM.COF");

        RadarPositionNavigationController radarPositionNavigationController(config.kalmanValues,
                                                                            databaseManager,
                                                                            std::move(imuSerialPortReader),
                                                                            std::move(imuManager));

        radarPositionNavigationController.StartAndConfigureRadarPNT(30.274137, -97.734889);
        
        std::cout << "Type anything to stop" << std::endl;
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
}
