#ifndef INU_DISPLAY_SQLITEMANAGER_HPP
#define INU_DISPLAY_SQLITEMANAGER_HPP

#include "GpsRecords.hpp"
#include "ImuRecords.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct sqlite3;
struct sqlite3_stmt;

class SqliteManager {
public:
    explicit SqliteManager(const std::string& dbPath);
    ~SqliteManager();

    SqliteManager(const SqliteManager&) = delete;
    SqliteManager& operator=(const SqliteManager&) = delete;

    void persistGyroscope(const std::vector<GyroscopeRecord>& batch);
    void persistAccelerometer(const std::vector<AccelerometerRecord>& batch);
    void persistGps(const std::vector<GpsRecord>& batch);

private:
    struct Sqlite3Deleter {
        void operator()(sqlite3* db) const noexcept;
    };
    struct StmtDeleter {
        void operator()(sqlite3_stmt* stmt) const noexcept;
    };

    using DbHandle = std::unique_ptr<sqlite3, Sqlite3Deleter>;
    using StmtHandle = std::unique_ptr<sqlite3_stmt, StmtDeleter>;

    void exec(const char* sql);
    StmtHandle prepare(const char* sql);

    DbHandle m_db;
    StmtHandle m_gyroInsert;
    StmtHandle m_accelInsert;
    StmtHandle m_gpsInsert;
    std::mutex m_mutex;
};

#endif
