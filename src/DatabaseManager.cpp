#include "DatabaseManager.hpp"

#include <iostream>
#include <chrono>

DatabaseManager::DatabaseManager(const std::filesystem::path &databasePath)
    : m_databasePath(databasePath),
      m_running(false),
      m_sqliteConnection(databasePath.string(), SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE) {
    InitializeSchema();
    PrepareSqlStmts();
}

DatabaseManager::~DatabaseManager() {
    Stop();
}

void DatabaseManager::Start() {
    if (m_running.exchange(true)) {
        return;
    }

    m_writerThread = std::jthread([this](std::stop_token st)
                                  { WriterLoop(st); });
}

void DatabaseManager::Stop() {
    if (!m_running.exchange(false)) {
        return;
    }

    m_writerThread.request_stop();
    m_queueCondition.notify_all();

    if (m_writerThread.joinable()) {
        m_writerThread.join();
    }

}

bool DatabaseManager::IsRunning() const {
    return m_running.load();
}

void DatabaseManager::EnqueueGpsUpdate(const GpsUpdate &update) {
    {
        std::lock_guard lock(m_stateMutex);
        m_recordQueue.emplace_back(update);
        m_stats.currentQueueDepth = m_recordQueue.size();
    }
    m_stats.gpsQueued++;
    m_queueCondition.notify_one();
}

void DatabaseManager::EnqueueIMULinearAcceleration(const sh2_SensorValue &measurement){
    {
        std::lock_guard lock(m_stateMutex);
        m_recordQueue.emplace_back(IMULinearAccelerationRecord(measurement));
        m_stats.currentQueueDepth = m_recordQueue.size();
    }
    m_stats.imuLinearAccelerationQueued++;
    m_queueCondition.notify_one();
}

void DatabaseManager::EnqueueIMURotationVector(const sh2_SensorValue &measurement) {
    {
        std::lock_guard lock(m_stateMutex);
        m_recordQueue.emplace_back(IMURotationVectorRecord(measurement));
        m_stats.currentQueueDepth = m_recordQueue.size();
    }
    m_stats.imuRotationVectorQueued++;
    m_queueCondition.notify_one();
}

void DatabaseManager::EnqueueEkfOutput(const Vector6d& v, const Matrix6d& m) {
    {
        std::lock_guard lock(m_stateMutex);
        m_recordQueue.emplace_back(EkfOutputRecord(v, m));
        m_stats.currentQueueDepth = m_recordQueue.size();
    }
    m_stats.ekfQueued++;
    m_queueCondition.notify_one();
}

DatabaseManagerStats DatabaseManager::GetStats() const {
    return m_stats;
}

void DatabaseManager::WriterLoop(std::stop_token stopToken) {
    std::vector<DatabaseRecord> batch;
    batch.reserve(4096);

    while (!stopToken.stop_requested()) {
        {
            std::unique_lock lock(m_stateMutex);

            m_queueCondition.wait_for(lock,
                                      std::chrono::milliseconds(10),
                                      [&] { return stopToken.stop_requested() ||
                                           !m_recordQueue.empty();
                                       });
            std::swap(batch, m_recordQueue);
        }

        if (!batch.empty()) {
            WriteBatch(batch);
            batch.clear();
        }
    }

    // flush
    {
        std::lock_guard lock(m_stateMutex);
        std::swap(batch, m_recordQueue);
    }

    if (!batch.empty()) {
        WriteBatch(batch);
    }
}

void DatabaseManager::InitializeSchema() {
    m_sqliteConnection.exec(R"(
        CREATE TABLE IF NOT EXISTS gps_update (
            timestamp REAL,
            latitude REAL,
            longitude REAL,
            heading REAL,
            fix_quality INTEGER,
            num_satellites INTEGER,
            hdop REAL,
            gps_timestamp_ms INTEGER,
            valid INTEGER
        );

        CREATE TABLE IF NOT EXISTS imu_linear_accel (
            timestamp REAL,
            status INTEGER,
            hw_timestamp_us INTEGER,
            x REAL,
            y REAL,
            z REAL
        );

        CREATE TABLE IF NOT EXISTS imu_rotation_vector (
            timestamp REAL,
            status INTEGER,
            hw_timestamp_us INTEGER,
            i REAL,
            j REAL,
            k REAL,
            real REAL,
            accuracy REAL
        );

        CREATE TABLE IF NOT EXISTS ekf_output (
            timestamp REAL,
            x REAL,
            y REAL,
            vx REAL,
            vy REAL,
            ax REAL,
            ay REAl,
            cov_mat_str TEXT
        );
    )");
}

void DatabaseManager::PrepareSqlStmts() {
    m_gpsStmt = std::make_unique<SQLite::Statement>(m_sqliteConnection,
                                                    R"(INSERT INTO gps_update
                                                (   timestamp,
                                                    latitude,
                                                    longitude,
                                                    heading,
                                                    fix_quality,
                                                    num_satellites,
                                                    hdop,
                                                    gps_timestamp_ms,
                                                    valid)
                                                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?))");

    m_laStmt = std::make_unique<SQLite::Statement>(m_sqliteConnection,
                                                   R"(INSERT INTO imu_linear_accel
                                                    (timestamp, status, hw_timestamp_us, x, y, z)
                                                    VALUES (?, ?, ?, ?, ?, ?))");

    m_rvStmt = std::make_unique<SQLite::Statement>(m_sqliteConnection,
                                                   R"(INSERT INTO imu_rotation_vector
                                                    (timestamp, status, hw_timestamp_us, i, j, k, real, accuracy)
                                                    VALUES (?, ?, ?, ?, ?, ?, ?, ?))");

    m_ekfStmt = std::make_unique<SQLite::Statement>(m_sqliteConnection,
                                                    R"(INSERT INTO ekf_output
                                                    (timestamp, x, y, vx, vy, ax, ay, cov_mat_str)
                                                    VALUES (?, ?, ?, ?, ?, ?, ?, ?))");
}

void DatabaseManager::WriteBatch(std::vector<DatabaseRecord> &batch) {
    try {
        SQLite::Transaction txn(m_sqliteConnection);

        for (auto &record : batch) {
            std::visit([&](auto &r) {
                using T = std::decay_t<decltype(r)>;

                if constexpr (std::is_same_v<T, GpsUpdate>)
                {
                    m_gpsStmt->reset();
                    m_gpsStmt->clearBindings();

                    m_gpsStmt->bind(1, r.timestamp);
                    m_gpsStmt->bind(2, r.latitude);
                    m_gpsStmt->bind(3, r.longitude);

                    if (r.heading.has_value())
                        m_gpsStmt->bind(4, *r.heading);
                    else
                        m_gpsStmt->bind(4);

                    m_gpsStmt->bind(5, static_cast<int>(r.fixQuality));
                    m_gpsStmt->bind(6, static_cast<int>(r.numSatellites));
                    m_gpsStmt->bind(7, r.hdop);
                    m_gpsStmt->bind(8, static_cast<int64_t>(r.gpsTimestampMs));
                    m_gpsStmt->bind(9, static_cast<int>(r.valid));
                    m_gpsStmt->exec();

                    m_stats.gpsWritten++;
                }

                else if constexpr (std::is_same_v<T, IMULinearAccelerationRecord>)
                {
                    m_laStmt->reset();
                    m_laStmt->clearBindings();

                    m_laStmt->bind(1, r.timestamp);
                    m_laStmt->bind(2, r.status);
                    m_laStmt->bind(3, static_cast<int64_t>(r.hwTimestampUs));
                    m_laStmt->bind(4, r.x);
                    m_laStmt->bind(5, r.y);
                    m_laStmt->bind(6, r.z);
                    m_laStmt->exec();

                    m_stats.imuLinearAccelerationWritten++;
                }

                else if constexpr (std::is_same_v<T, IMURotationVectorRecord>)
                {
                    m_rvStmt->reset();
                    m_rvStmt->clearBindings();

                    m_rvStmt->bind(1, r.timestamp);
                    m_rvStmt->bind(2, r.status);
                    m_rvStmt->bind(3, static_cast<int64_t>(r.hwTimestampUs));
                    m_rvStmt->bind(4, r.i);
                    m_rvStmt->bind(5, r.j);
                    m_rvStmt->bind(6, r.k);
                    m_rvStmt->bind(7, r.real);
                    m_rvStmt->bind(8, r.accuracy);
                    m_rvStmt->exec();

                    m_stats.imuRotationVectorWritten++;
                }

                else if constexpr (std::is_same_v<T, EkfOutputRecord>)
                {
                    m_ekfStmt->reset();
                    m_ekfStmt->clearBindings();

                    m_ekfStmt->bind(1, r.timestamp);
                    m_ekfStmt->bind(2, r.x);
                    m_ekfStmt->bind(3, r.y);
                    m_ekfStmt->bind(4, r.vx);
                    m_ekfStmt->bind(5, r.vy);
                    m_ekfStmt->bind(6, r.ax);
                    m_ekfStmt->bind(7, r.ay);
                    m_ekfStmt->bind(8, r.covMatStr);
                    m_ekfStmt->exec();

                    m_stats.ekfWritten++;
                }
            },
            record);
        }

        txn.commit();
    }
    catch (const std::exception &e)
    {
        m_stats.failedWrites++;
        std::cerr << "Database WriteBatch failed: " << e.what() << "\n";
    }
}