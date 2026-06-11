#ifndef DATABASE_MANAGER_HPP
#define DATABASE_MANAGER_HPP

#include <queue>
#include <thread>
#include <memory>
#include <vector>
#include <variant>
#include <optional>
#include <filesystem>
#include <condition_variable>

#include <SQLiteCpp/SQLiteCpp.h>

#include "GpsUpdate.hpp"
#include "DatabaseManagerStats.hpp"
#include "IMULinearAccelerometerRecord.hpp"
#include "IMURotationVectorRecord.hpp"
#include "EkfOutputRecord.hpp"

using DatabaseRecord = std::variant<GpsUpdate,
                                    IMULinearAccelerationRecord,
                                    IMURotationVectorRecord,
                                    EkfOutputRecord>;

class DatabaseManager {

public:
    /**
     * @brief constructor
     * 
     * @param databasePath a string that points to the *.db file
     * 
     * @return
     */
    DatabaseManager(const std::filesystem::path& databasePath);

    /**
     * @brief destructor
     * 
     * @return
     */
    ~DatabaseManager();

    /**
     * @brief stops and join m_writerThread
     * 
     * @return
     */
    void Stop();

    /**
     * @brief starts m_writerThread once
     * 
     * @return
     */
    void Start();

    /**
     * @brief getter of m_running
     * 
     * @return true if m_running == true, else false
     */
    bool IsRunning() const;

    void EnqueueGpsUpdate(const GpsUpdate& update);
    void EnqueueEkfOutput(const EkfOutputRecord& output);
    void EnqueueIMURotationVector(const sh2_SensorValue& measurement);
    void EnqueueIMULinearAcceleration(const sh2_SensorValue& measurement);

    DatabaseManagerStats GetStats() const;

private:
    void PrepareSqlStmts();
    void InitializeSchema();
    void WriterLoop(std::stop_token stopToken);
    void WriteBatch(std::vector<DatabaseRecord>& batch);

    
    std::atomic<bool> m_running;
    std::jthread m_writerThread;

    std::mutex m_stateMutex;
    std::queue<DatabaseRecord> m_recordQueue;
    std::condition_variable_any m_queueCondition;
    
    DatabaseManagerStats m_stats;
    mutable std::mutex m_statsMutex;
    
    SQLite::Database m_sqliteConnection;
    std::filesystem::path m_databasePath;

    std::unique_ptr<SQLite::Statement> m_gpsStmt;
    std::unique_ptr<SQLite::Statement> m_laStmt;
    std::unique_ptr<SQLite::Statement> m_rvStmt;
    std::unique_ptr<SQLite::Statement> m_ekfStmt;
};

#endif