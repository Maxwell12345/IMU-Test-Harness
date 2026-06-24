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
        std::cout << rot.value().timestamp << " " << rot.value().i << " " << rot.value().j << " " << rot.value().k << " " << rot.value().accuracy << std::endl;
    }
    else if (acc.has_value()) {
        std::cout << acc.value().timestamp << " " << acc.value().x << " " << acc.value().y << " " << acc.value().z << std::endl;
    }
}

int main(int argc,char** argv) {
    try {
        IMUSerialPortReader reader = IMUSerialPortReader("/dev/ttyUSB0", 115200);

        reader.InstallVectorCallback(&callback);

        reader.Start();

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
