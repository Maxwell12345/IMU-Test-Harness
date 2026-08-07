/******************************************************************************
 * File:             GpsManager.hpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 * Description:      Declares GPS update delivery and the reusable mapping from
 *                   parsed NMEA messages to timestamped GpsUpdate objects.
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
#include <cstdint>
#include <mutex>

class GpsManager: public GpsManagerBase {
public:
    GpsManager();
    
    void InstallCallback(std::function<void(const GpsUpdate&)> callback) override;
    void Start() override;
    void Stop()  override;

    /**
     *
     * @brief   Maps a parsed NMEA message and its database capture timestamp into a GpsUpdate suitable for IMUManager. The
     *          database timestamp is retained for chronological ordering while replay delivery time is recorded separately
     *          for the existing freshness check.
     *
     * @param [in] msg                  Parsed RMC or GGA NMEA message containing coordinates and validity information.
     * @param [in] databaseTimestampNs  Recorded host capture timestamp in nanoseconds. Its millisecond representation must fit
     *                                  in the GpsUpdate::gpsTimestampMs field.
     *
     * @return  GpsUpdate containing coordinates, validity, optional RMC course, replay receive time, and the recorded database
     *          timestamp expressed in seconds and milliseconds.
     *
     * @throws  std::out_of_range  If databaseTimestampNs expressed in milliseconds exceeds uint32_t.
     */
    static GpsUpdate BuildGpsUpdate(const NmeaMessage& msg, uint64_t databaseTimestampNs);

private:
    std::function<void(const GpsUpdate&)> m_callback;
    std::mutex m_callbackMutex;
    bool m_isCallbackInstalled = false;

    std::atomic<bool> m_running = false;
    std::thread m_thread;

    NmeaReader m_nmeaReader;
};

#endif // INU_DISPLAY_GPSMANAGER_HPP
