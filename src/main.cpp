// #include "IMUManager.hpp"
// #include "IMUGPSFusionKF.hpp"
// #include "DatabaseManager.hpp"
// #include "RadarPositionNavigationController.hpp"

// #include "demo.hpp"

// #include <atomic>
// #include <chrono>
// #include <iostream>
// #include <mutex>
// #include <thread>
// #include <termios.h>
// #include <unistd.h>
// #include <fstream>

// #include "bno085_hal.hpp"

// #include "imu_data.hpp"


// std::atomic<bool> g_running {true};

// int main() {
//     std::shared_ptr<DatabaseManager> db = std::make_shared<DatabaseManager>("./IMUPROC.db");
//     db->Start();

//     RadarPositionNavigationController radarPositionNavigationController(db);

//     while (g_running) {
//         char ch = 0;
//         if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1B') {
//             g_running = false;
//             break;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     std::cout << "\nShutdown complete.\n";
// }


#include <thread>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include <boost/asio.hpp>

#include "utils.hpp"
#include "SerialComService.hpp"
#include "IMUSerialPortReader.hpp"
#include "SerialPort/BoostSerialPort.hpp"

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
            IMUUtils::LOG_DEBUG(line, true);
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
    } catch (std::runtime_error &e) {
        IMUUtils::LOG_ERROR(e.what());
    }
}
