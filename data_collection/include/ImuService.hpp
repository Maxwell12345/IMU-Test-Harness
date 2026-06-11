#ifndef INU_DISPLAY_IMUSERVICE_HPP
#define INU_DISPLAY_IMUSERVICE_HPP

#include <atomic>
#include <thread>

extern "C" {
#include "sh2.h"
}

class BrokerService;

class ImuService {
public:
    explicit ImuService(BrokerService& broker);
    ~ImuService();

    ImuService(const ImuService&) = delete;
    ImuService& operator=(const ImuService&) = delete;

    void start();
    void stop();

private:
    static void sensorCallback(void* cookie, sh2_SensorEvent_t* event);
    void handleSensorEvent(sh2_SensorEvent_t* event);
    void serviceLoop();

    BrokerService& m_broker;
    sh2_Hal_t m_hal{};
    std::thread m_serviceThread;
    std::atomic<bool> m_running{false};
};

#endif
