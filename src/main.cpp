#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include <boost/asio.hpp>

#include "utils.hpp"
#include "SerialComService.hpp"
#include "IMUSerialPortReader.hpp"

void callback(std::optional<Raw_RotationVectorWAcc> rot, std::optional<Raw_Accelerometer> acc) {
    if (rot.has_value()) {
        std::cout << "ROT: " << rot.value().timestamp << " " << rot.value().i << " " << rot.value().j << " " << rot.value().k << " " << rot.value().accuracy << std::endl;
    }
    else if (acc.has_value()) {
        std::cout << "ACC: " << acc.value().timestamp << " " << acc.value().x << " " << acc.value().y << " " << acc.value().z << std::endl;
    }
}

int main(int argc, char** argv) {
    try {
        IMUSerialPortReader reader("/dev/ttyUSB0", 115200);

        reader.InstallVectorCallback(callback);

        reader.Start();

        std::signal(SIGINT, [](int) {
            std::exit(EXIT_SUCCESS);
        });

        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        std::cout << "[ERROR] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
