#include "IMUManager.hpp"
#include "IMUGPSFusionKF.hpp"
#include "DatabaseManager.hpp"
#include "RadarPositionNavigationController.hpp"

#include "demo.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fstream>

#include "bno085_hal.hpp"

extern "C" {
#include "sh2_SensorValue.h"
#include "sh2.h"
}

#include "imu_data.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

std::atomic<bool> g_running {true};

int main() {
    boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>("./IMUPROC.db");
    db->Start();

    RadarPositionNavigationController radarPositionNavigationController(db);

    while (g_running) {
        char ch = 0;
        if (::read(STDIN_FILENO, &ch, 1) == 1 && ch == '\x1B') {
            g_running = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nShutdown complete.\n";
}