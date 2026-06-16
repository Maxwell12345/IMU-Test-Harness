#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>

#include "GpsService.hpp"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main(int argc,char** argv) {
    std::signal(SIGINT, signalHandler);

    GpsService gpsService("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_A7CMb151406-if00-port0", 115200);
    gpsService.Start();

    std::cout << "\nPress ctrl + c to stop application\n\n";
    while(keepRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gpsService.Stop();

    return EXIT_SUCCESS;
}
