#include "RadarPositionNavigationController.hpp"
#include "gps/GpsManager.hpp"
#include "IMUManager.hpp"
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <iostream>
#include <thread>


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    auto databaseManager = boost::make_shared<DatabaseManager>("./log_db");

    databaseManager->Start();

    IMUManager _ = IMUManager(databaseManager);
    RadarPositionNavigationController controller = RadarPositionNavigationController();
    GpsManager gpsManager(databaseManager);

    gpsManager.InstallCallback(controller.GetGPSCallback());
    gpsManager.Start();

    bool hasPosition = false;

    while (!hasPosition) {
        auto optPos = gpsManager.GetCurrentPosition();

        if (optPos == std::nullopt) {
            std::cout << "Didn't find GPS fix" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            continue;
        }

        controller.StartAndConfigureRadarPNT(optPos->first, optPos->second);
        hasPosition = true;
        break;
    }

    std::cout << "Press Enter to stop processor" << std::endl;
    std::cin.get();

    auto s = databaseManager->GetStats();
    std::cout << "ekfQueued=" << s.ekfQueued
            << " ekfWritten=" << s.ekfWritten
            << " failedWrites=" << s.failedWrites
            << " queueDepth=" << s.currentQueueDepth << std::endl;

    gpsManager.Stop();
    controller.TotalDestruction();
    databaseManager->Stop();

    return 0;
}