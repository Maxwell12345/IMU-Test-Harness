#include "RadarPositionNavigationController.hpp"
#include "gps/GpsManager.hpp"
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <iostream>
#include <thread>


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    auto databaseManager = boost::make_shared<DatabaseManager>("./log_db");

    RadarPositionNavigationController controller;
    GpsManager gpsManager(databaseManager);

    gpsManager.InstallCallback(controller.GetGPSCallback());
    gpsManager.Start();

    bool hasPosition = false;

    while (!hasPosition) {
        auto optPos = gpsManager.GetCurrentPosition();

        if (optPos == std::nullopt) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        controller.StartAndConfigureRadarPNT(optPos->first, optPos->second);
        hasPosition = true;
    }

    std::cout << "Press Enter to stop processor" << std::endl;
    std::cin.get();

    gpsManager.Stop();
    controller.TotalDestruction();

    return 0;
}