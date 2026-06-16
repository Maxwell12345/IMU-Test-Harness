#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include "Utils.hpp"
#include "GpsService.hpp"
#include "SerialPort/BoostSerialPort.hpp"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main(int argc,char** argv) {
    try{
        std::signal(SIGINT, signalHandler);

        std::string path = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_A7CMb151406-if00-port0";
        GpsService gpsService(path,
                              115200,
                              std::make_unique<BoostSerialPort>());
        gpsService.Start();

        std::cout << "\nPress ctrl + c to stop application\n\n";
        while(keepRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        gpsService.Stop();

        return EXIT_SUCCESS;
    } catch (std::runtime_error &e) {
        utils::LOG_ERROR(e.what());
    }
}
