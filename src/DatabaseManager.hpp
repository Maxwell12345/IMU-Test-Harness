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
#include "IMULinearAccelerationRecord.hpp"
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

    /**
     * @brief Enqueue GPS data to m_recordQueue
     * 
     * @param [in] update gps update value
     * 
     * @return
     */
    void EnqueueGpsUpdate(const GpsUpdate& update);

    void EnqueueEkfOutput(const EkfOutputRecord& output);

    /**
     * @brief Enqueue SH2_ROTATION_VECTOR sensor output to m_recordQueue
     * 
     * @remarks converts sh2_SensorValue to IMURotationVectorRecord
     * 
     * @param [in] measurement sensor value
     * 
     * @return 
     */
    void EnqueueIMURotationVector(const sh2_SensorValue& measurement);

    /**
     * @brief Enqueue SH2_LINEAR_ACCELERATION sensor output to m_recordQueue
     * 
     * @remarks converts sh2_SensorValue to IMULinearAccelerationVectorRecord
     * 
     * @param [in] measurement sensor value
     * 
     * @return 
     */
    void EnqueueIMULinearAcceleration(const sh2_SensorValue& measurement);

    /**
     * @brief accessor to atomic number of items in queue, written items, failed writes
     * 
     * @return DatabaseManagerStats snapshot
     */
    DatabaseManagerStats GetStats() const;

private:
    /**
     * @brief helper function called in constructor to instantiate SQLite::Statement.
     *      Decreases overhead compared to creating every db write.
     * 
     * @return
     */
    void PrepareSqlStmts();

    /**
     * @brief helper function to create tables
     * 
     * @return
     */
    void InitializeSchema();

    /**
     * @brief main write loop
     * 
     * @remarks loop will initially obtain a lock on m_stateMutex, then it will release it
     *      for producers. Loop will exit if request_stop() is called. If m_recordQueue is
     *      not empty then the loop will start batching and writing the batch. If loop
     *      stops, it will flush the remaining item from the queue and write what is left
     *      in the batch
     * 
     * @param [in] stopToken callback parameter used to determine if request_stop() has
     *      been invoked
     */
    void WriterLoop(std::stop_token stopToken);

    /**
     * @brief batch write to db
     * 
     * @remarks iterates over the batched variants, compare variant type, binds sql
     *      statements, then exec.
     * 
     * @param [in] batch variant records to be writen to db
     * 
     * @return
     */
    void WriteBatch(std::vector<DatabaseRecord>& batch);

    std::atomic<bool> m_running;
    std::jthread m_writerThread;

    std::mutex m_stateMutex;
    std::vector<DatabaseRecord> m_recordQueue;
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