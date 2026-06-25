#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>

#include "NmeaService.hpp"
#include "BoostSerialPort.hpp"
#include "MulticastService.hpp"
#include "SerialComService.hpp"
#include "YamlConfigService.hpp"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
    std::cout << "\n[Interrupt] Cleanup started..." << std::endl;
    keepRunning = false;
}

int main() {
    try {
        YamlConfigService yamlConfigService("config.yaml");
        auto config = yamlConfigService.GetConfig();
        std::cout << "[INFO] Yaml config\n" << config.ToString() << std::endl;

        std::signal(SIGINT, signalHandler);

        auto serialComService = std::make_unique<SerialComService>(config.nmeaConfig.serialPortPath,
                                                                   config.nmeaConfig.serialPortBaudRate,
                                                                   std::make_unique<BoostSerialPort>());

        auto multicastService = std::make_unique<MulticastService>(config.nmeaConfig.multicastIp,
                                                                   config.nmeaConfig.multicastPort);
        NmeaService nmeaService(std::move(serialComService),
                                std::move(multicastService));

        nmeaService.Start();

        std::cout << "\nPress ctrl + c to stop application\n\n";
        while(keepRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        nmeaService.Stop();

        return EXIT_SUCCESS;
    } catch(const std::invalid_argument &e) {
        std::cout << "[ERROR]" << e.what() << std::endl;
    } catch (const std::runtime_error &e) {
        std::cout << "[ERROR]" << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}