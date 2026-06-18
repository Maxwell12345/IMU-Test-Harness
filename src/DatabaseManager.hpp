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
#include "EkfOutputRecord.hpp"
#include "imu_data.hpp"
#include "DatabaseManagerStats.hpp"
#include "IMURotationVectorRecord.hpp"
#include "IMULinearAccelerationRecord.hpp"

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

    void EnqueueEkfOutput(const Vector6d& v, const Matrix6d& m);

    /**
     * @brief Enqueue rotation vector sensor output to m_recordQueue
     * 
     * @remarks converts Raw_RotationVectorWAcc to IMURotationVectorRecord
     * 
     * @param [in] rv sensor value
     * 
     * @return 
     */
    void EnqueueIMURotationVector(const Raw_RotationVectorWAcc& rv);

    /**
     * @brief Enqueue Linear acceleration sensor output to m_recordQueue
     * 
     * @remarks converts Raw_Accelerometer to IMULinearAccelerationVectorRecord
     * 
     * @param [in] la sensor value
     * 
     * @return 
     */
    void EnqueueIMULinearAcceleration(const Raw_Accelerometer& la);

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

    std::atomic<bool> m_running;                    // atomic running flag
    std::jthread m_writerThread;                    // joinable run thread

    std::mutex m_stateMutex;                        // mutex that protects read / write to queue
    std::vector<DatabaseRecord> m_recordQueue;      // acts as queue, supports std::swap
    std::condition_variable_any m_queueCondition;   // conditional used with timeout of 10ms as well
    
    DatabaseManagerStats m_stats;                   // internal statistic tracking
    
    SQLite::Database m_sqliteConnection;
    std::filesystem::path m_databasePath;

    std::unique_ptr<SQLite::Statement> m_gpsStmt;   // Ptr to preparable stmts, decrease overhead during run time
    std::unique_ptr<SQLite::Statement> m_laStmt;
    std::unique_ptr<SQLite::Statement> m_rvStmt;
    std::unique_ptr<SQLite::Statement> m_ekfStmt;
};

#endif