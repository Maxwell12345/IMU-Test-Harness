#include "SqliteManager.hpp"

#include "Logger.hpp"

#include <sqlite3.h>

#include <stdexcept>

namespace {

template <typename T>
void bindBatch(sqlite3* db, sqlite3_stmt* stmt, const std::vector<T>& batch) {
    for (const T& record : batch) {
        sqlite3_bind_int64(stmt, 1, record.timestampNs);
        sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(record.sensorTimestampUs));
        sqlite3_bind_double(stmt, 3, record.x);
        sqlite3_bind_double(stmt, 4, record.y);
        sqlite3_bind_double(stmt, 5, record.z);
        sqlite3_bind_int(stmt, 6, record.calibAccuracy);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            sqlite3_reset(stmt);
            throw std::runtime_error(std::string("SQLite step failed: ") +
                                     sqlite3_errmsg(db));
        }
        sqlite3_reset(stmt);
    }
}

}  // namespace

void SqliteManager::Sqlite3Deleter::operator()(sqlite3* db) const noexcept {
    sqlite3_close(db);
}

void SqliteManager::StmtDeleter::operator()(sqlite3_stmt* stmt) const noexcept {
    sqlite3_finalize(stmt);
}

SqliteManager::SqliteManager(const std::string& dbPath) {
    sqlite3* raw = nullptr;
    if (sqlite3_open(dbPath.c_str(), &raw) != SQLITE_OK) {
        const std::string message = raw ? sqlite3_errmsg(raw) : "out of memory";
        sqlite3_close(raw);
        throw std::runtime_error("Failed to open SQLite database: " + message);
    }
    m_db.reset(raw);

    exec("PRAGMA journal_mode=WAL;");
    exec("PRAGMA synchronous=NORMAL;");

    exec("CREATE TABLE IF NOT EXISTS gyroscope ("
         "timestamp_ns INTEGER NOT NULL, "
         "sensor_timestamp_us INTEGER NOT NULL, "
         "gyro_x REAL NOT NULL, "
         "gyro_y REAL NOT NULL, "
         "gyro_z REAL NOT NULL, "
         "calib_accuracy INTEGER NOT NULL);");

    exec("CREATE TABLE IF NOT EXISTS accelerometer ("
         "timestamp_ns INTEGER NOT NULL, "
         "sensor_timestamp_us INTEGER NOT NULL, "
         "acc_x REAL NOT NULL, "
         "acc_y REAL NOT NULL, "
         "acc_z REAL NOT NULL, "
         "calib_accuracy INTEGER NOT NULL);");

    exec("CREATE TABLE IF NOT EXISTS linear_acceleration ("
         "timestamp_ns INTEGER NOT NULL, "
         "sensor_timestamp_us INTEGER NOT NULL, "
         "lin_acc_x REAL NOT NULL, "
         "lin_acc_y REAL NOT NULL, "
         "lin_acc_z REAL NOT NULL, "
         "calib_accuracy INTEGER NOT NULL);");

    exec("CREATE TABLE IF NOT EXISTS rotation_vector ("
         "timestamp_ns INTEGER NOT NULL, "
         "sensor_timestamp_us INTEGER NOT NULL, "
         "quat_i REAL NOT NULL, "
         "quat_j REAL NOT NULL, "
         "quat_k REAL NOT NULL, "
         "quat_real REAL NOT NULL, "
         "accuracy_rad REAL NOT NULL, "
         "calib_accuracy INTEGER NOT NULL);");

    exec("CREATE TABLE IF NOT EXISTS gps ("
         "timestamp_ns INTEGER NOT NULL, "
         "device_id TEXT NOT NULL, "
         "nmea_code TEXT NOT NULL, "
         "nmea_raw TEXT NOT NULL, "
         "latitude REAL, "
         "longitude REAL, "
         "altitude REAL, "
         "satellites INTEGER);");

    m_gyroInsert = prepare("INSERT INTO gyroscope "
                           "(timestamp_ns, sensor_timestamp_us, gyro_x, gyro_y, gyro_z, "
                           "calib_accuracy) "
                           "VALUES (?, ?, ?, ?, ?, ?);");
    m_accelInsert = prepare("INSERT INTO accelerometer "
                            "(timestamp_ns, sensor_timestamp_us, acc_x, acc_y, acc_z, "
                            "calib_accuracy) "
                            "VALUES (?, ?, ?, ?, ?, ?);");
    m_linAccInsert = prepare("INSERT INTO linear_acceleration "
                             "(timestamp_ns, sensor_timestamp_us, lin_acc_x, lin_acc_y, "
                             "lin_acc_z, calib_accuracy) "
                             "VALUES (?, ?, ?, ?, ?, ?);");
    m_rotInsert = prepare("INSERT INTO rotation_vector "
                          "(timestamp_ns, sensor_timestamp_us, quat_i, quat_j, quat_k, "
                          "quat_real, accuracy_rad, calib_accuracy) "
                          "VALUES (?, ?, ?, ?, ?, ?, ?, ?);");
    m_gpsInsert = prepare("INSERT INTO gps "
                          "(timestamp_ns, device_id, nmea_code, nmea_raw, "
                          "latitude, longitude, altitude, satellites) "
                          "VALUES (?, ?, ?, ?, ?, ?, ?, ?);");

    LOG_INFO("SqliteManager") << "Opened database '" << dbPath
                              << "' (tables: gyroscope, accelerometer, linear_acceleration, "
                                 "rotation_vector, gps; WAL mode)";
}

SqliteManager::~SqliteManager() = default;

void SqliteManager::exec(const char* sql) {
    char* errorMessage = nullptr;
    if (sqlite3_exec(m_db.get(), sql, nullptr, nullptr, &errorMessage) != SQLITE_OK) {
        const std::string message = errorMessage ? errorMessage : "unknown error";
        sqlite3_free(errorMessage);
        throw std::runtime_error("SQLite exec failed: " + message);
    }
}

SqliteManager::StmtHandle SqliteManager::prepare(const char* sql) {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(m_db.get(), sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(std::string("SQLite prepare failed: ") +
                                 sqlite3_errmsg(m_db.get()));
    }
    return StmtHandle(stmt);
}

void SqliteManager::persistGyroscope(const std::vector<GyroscopeRecord>& batch) {
    if (batch.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    exec("BEGIN TRANSACTION;");
    try {
        bindBatch(m_db.get(), m_gyroInsert.get(), batch);
    } catch (...) {
        exec("ROLLBACK;");
        throw;
    }
    exec("COMMIT;");
    LOG_DEBUG("SqliteManager") << "Persisted " << batch.size() << " gyroscope rows";
}

void SqliteManager::persistAccelerometer(const std::vector<AccelerometerRecord>& batch) {
    if (batch.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    exec("BEGIN TRANSACTION;");
    try {
        bindBatch(m_db.get(), m_accelInsert.get(), batch);
    } catch (...) {
        exec("ROLLBACK;");
        throw;
    }
    exec("COMMIT;");
    LOG_DEBUG("SqliteManager") << "Persisted " << batch.size() << " accelerometer rows";
}

void SqliteManager::persistLinearAccel(const std::vector<LinearAccelRecord>& batch) {
    if (batch.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    exec("BEGIN TRANSACTION;");
    try {
        bindBatch(m_db.get(), m_linAccInsert.get(), batch);
    } catch (...) {
        exec("ROLLBACK;");
        throw;
    }
    exec("COMMIT;");
    LOG_DEBUG("SqliteManager") << "Persisted " << batch.size() << " linear_acceleration rows";
}

void SqliteManager::persistRotationVector(const std::vector<RotationVectorRecord>& batch) {
    if (batch.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    exec("BEGIN TRANSACTION;");
    try {
        sqlite3_stmt* stmt = m_rotInsert.get();
        for (const RotationVectorRecord& record : batch) {
            sqlite3_bind_int64(stmt, 1, record.timestampNs);
            sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(record.sensorTimestampUs));
            sqlite3_bind_double(stmt, 3, record.i);
            sqlite3_bind_double(stmt, 4, record.j);
            sqlite3_bind_double(stmt, 5, record.k);
            sqlite3_bind_double(stmt, 6, record.real);
            sqlite3_bind_double(stmt, 7, record.accuracyRad);
            sqlite3_bind_int(stmt, 8, record.calibAccuracy);

            if (sqlite3_step(stmt) != SQLITE_DONE) {
                sqlite3_reset(stmt);
                throw std::runtime_error(std::string("SQLite step failed: ") +
                                         sqlite3_errmsg(m_db.get()));
            }
            sqlite3_reset(stmt);
        }
    } catch (...) {
        exec("ROLLBACK;");
        throw;
    }
    exec("COMMIT;");
    LOG_DEBUG("SqliteManager") << "Persisted " << batch.size() << " rotation_vector rows";
}

void SqliteManager::persistGps(const std::vector<GpsRecord>& batch) {
    if (batch.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    exec("BEGIN TRANSACTION;");
    try {
        sqlite3_stmt* stmt = m_gpsInsert.get();
        for (const GpsRecord& record : batch) {
            sqlite3_bind_int64(stmt, 1, record.timestampNs);
            sqlite3_bind_text(stmt, 2, record.deviceId.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_text(stmt, 3, record.nmeaCode.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_text(stmt, 4, record.rawSentence.c_str(), -1, SQLITE_TRANSIENT);

            if (record.latitude) {
                sqlite3_bind_double(stmt, 5, *record.latitude);
            } else {
                sqlite3_bind_null(stmt, 5);
            }
            if (record.longitude) {
                sqlite3_bind_double(stmt, 6, *record.longitude);
            } else {
                sqlite3_bind_null(stmt, 6);
            }
            if (record.altitude) {
                sqlite3_bind_double(stmt, 7, *record.altitude);
            } else {
                sqlite3_bind_null(stmt, 7);
            }
            if (record.satellites) {
                sqlite3_bind_int(stmt, 8, *record.satellites);
            } else {
                sqlite3_bind_null(stmt, 8);
            }

            if (sqlite3_step(stmt) != SQLITE_DONE) {
                sqlite3_reset(stmt);
                throw std::runtime_error(std::string("SQLite step failed: ") +
                                         sqlite3_errmsg(m_db.get()));
            }
            sqlite3_reset(stmt);
        }
    } catch (...) {
        exec("ROLLBACK;");
        throw;
    }
    exec("COMMIT;");
    LOG_DEBUG("SqliteManager") << "Persisted " << batch.size() << " gps rows";
}
