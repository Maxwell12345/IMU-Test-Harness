/******************************************************************************
 * File:             GpsManager.hpp
 *
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 * Description:      Briefly describe the purpose of this file and its role within
 *                   the project. Mention key functions or classes it Contains.
 *
 ******************************************************************************/
#ifndef INU_DISPLAY_GPSMANAGER_HPP
#define INU_DISPLAY_GPSMANAGER_HPP

#include "NmeaReader.hpp"
#include "GpsUpdate.hpp"
#include "GpsManagerBase.hpp"

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

class GpsManager: public GpsManagerBase {
public:
    GpsManager();
    
    void InstallCallback(std::function<void(const GpsUpdate&)> callback) override;
    void Start() override;
    void Stop()  override;

private:
    GpsUpdate BuildGpsUpdate(const NmeaMessage& msg);

    std::function<void(const GpsUpdate&)> m_callback;
    std::mutex m_callbackMutex;
    bool m_isCallbackInstalled = false;

    std::atomic<bool> m_running = false;
    std::thread m_thread;

    NmeaReader m_nmeaReader;
};

#endif // INU_DISPLAY_GPSMANAGER_HPP
