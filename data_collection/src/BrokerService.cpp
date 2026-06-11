#include "BrokerService.hpp"

#include "Logger.hpp"

BrokerService::BrokerService(SqliteManager& sqliteManager, std::size_t batchSize)
    : m_sqliteManager(sqliteManager),
      m_gyroQueue("gyroscope", batchSize,
                  [this](std::vector<GyroscopeRecord>& batch) {
                      m_sqliteManager.persistGyroscope(batch);
                  }),
      m_accelQueue("accelerometer", batchSize,
                   [this](std::vector<AccelerometerRecord>& batch) {
                       m_sqliteManager.persistAccelerometer(batch);
                   }),
      m_gpsQueue("gps", batchSize,
                 [this](std::vector<GpsRecord>& batch) {
                     m_sqliteManager.persistGps(batch);
                 }) {}

void BrokerService::start() {
    LOG_INFO("BrokerService") << "Starting stream queues (batch size " << msDefaultBatchSize
                              << ")";
    m_gyroQueue.start();
    m_accelQueue.start();
    m_gpsQueue.start();
}

void BrokerService::stop() {
    LOG_INFO("BrokerService") << "Stopping stream queues and flushing remaining records";
    m_gyroQueue.stop();
    m_accelQueue.stop();
    m_gpsQueue.stop();
}

void BrokerService::enqueueGyroscope(const GyroscopeRecord& record) {
    m_gyroQueue.push(record);
}

void BrokerService::enqueueAccelerometer(const AccelerometerRecord& record) {
    m_accelQueue.push(record);
}

void BrokerService::enqueueGps(const GpsRecord& record) {
    m_gpsQueue.push(record);
}
