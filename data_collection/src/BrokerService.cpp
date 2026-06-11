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
      m_linAccQueue("linear_acceleration", batchSize,
                    [this](std::vector<LinearAccelRecord>& batch) {
                        m_sqliteManager.persistLinearAccel(batch);
                    }),
      m_rotQueue("rotation_vector", batchSize,
                 [this](std::vector<RotationVectorRecord>& batch) {
                     m_sqliteManager.persistRotationVector(batch);
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
    m_linAccQueue.start();
    m_rotQueue.start();
    m_gpsQueue.start();
}

void BrokerService::stop() {
    LOG_INFO("BrokerService") << "Stopping stream queues and flushing remaining records";
    m_gyroQueue.stop();
    m_accelQueue.stop();
    m_linAccQueue.stop();
    m_rotQueue.stop();
    m_gpsQueue.stop();
}

void BrokerService::enqueueGyroscope(const GyroscopeRecord& record) {
    m_gyroQueue.push(record);
}

void BrokerService::enqueueAccelerometer(const AccelerometerRecord& record) {
    m_accelQueue.push(record);
}

void BrokerService::enqueueLinearAccel(const LinearAccelRecord& record) {
    m_linAccQueue.push(record);
}

void BrokerService::enqueueRotationVector(const RotationVectorRecord& record) {
    m_rotQueue.push(record);
}

void BrokerService::enqueueGps(const GpsRecord& record) {
    m_gpsQueue.push(record);
}
