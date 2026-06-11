#ifndef INU_DISPLAY_BROKERSERVICE_HPP
#define INU_DISPLAY_BROKERSERVICE_HPP

#include "BatchQueue.hpp"
#include "GpsRecords.hpp"
#include "ImuRecords.hpp"
#include "SqliteManager.hpp"

#include <cstddef>

class BrokerService {
public:
    static constexpr std::size_t msDefaultBatchSize = 4096;

    explicit BrokerService(SqliteManager& sqliteManager,
                           std::size_t batchSize = msDefaultBatchSize);

    BrokerService(const BrokerService&) = delete;
    BrokerService& operator=(const BrokerService&) = delete;

    void start();
    void stop();

    void enqueueGyroscope(const GyroscopeRecord& record);
    void enqueueAccelerometer(const AccelerometerRecord& record);
    void enqueueLinearAccel(const LinearAccelRecord& record);
    void enqueueRotationVector(const RotationVectorRecord& record);
    void enqueueGps(const GpsRecord& record);

private:
    SqliteManager& m_sqliteManager;
    BatchQueue<GyroscopeRecord> m_gyroQueue;
    BatchQueue<AccelerometerRecord> m_accelQueue;
    BatchQueue<LinearAccelRecord> m_linAccQueue;
    BatchQueue<RotationVectorRecord> m_rotQueue;
    BatchQueue<GpsRecord> m_gpsQueue;
};

#endif
