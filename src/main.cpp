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

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main(int argc,char** argv) {
    try {
        std::signal(SIGINT, signalHandler);

        auto f = [](const GpsUpdate& g){
            printf("[DEBUG] Lat %0.5f Lon %0.5f\n", g.latitude, g.longitude);
        };
        GpsManager gpsManager;
        gpsManager.InstallCallback(f);
        gpsManager.Start();

        std::cout << "\nPress ctrl + c to stop application\n\n";
        while(keepRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        gpsManager.Stop();

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
