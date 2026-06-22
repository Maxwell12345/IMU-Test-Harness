#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include <boost/asio.hpp>

#include "utils.hpp"
#include "SerialComService.hpp"
#include "IMUSerialPortReader.hpp"
#include "BoostSerialPort.hpp"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main(int argc,char** argv) {
    try {
        std::signal(SIGINT, signalHandler);

        // Example call back for NMEA
        std::function f = [](boost::asio::serial_port& serial){
            boost::asio::streambuf buf;
            boost::asio::read_until(serial, buf, "\n");
            std::istream is(&buf);
            std::string line;
            std::getline(is, line);
            // TODO: LOG_DEBUG HERE
            std::cout << "[DEBUG]" << line << std::endl;
        };

        std::string path = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_A7CMb151406-if00-port0";
        SerialComService comService(path,
                                    115200,
                                    std::make_unique<BoostSerialPort>(f));
        comService.Start();

        std::cout << "\nPress ctrl + c to stop application\n\n";
        while(keepRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        comService.Stop();

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
