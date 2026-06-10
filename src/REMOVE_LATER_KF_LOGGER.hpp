#include <fstream>
#include <sstream>
#include <iomanip>
#include <future>
#include <mutex>
#include <vector>
#include <chrono>
#include <string>

static std::mutex g_kfCsvMutex;
static std::vector<std::string> g_kfCsvRows;
static std::future<void> g_kfCsvWriter;
static bool g_kfCsvHeaderWritten = false;
static uint64_t g_kfCsvStep = 0;

void LogKFCSV(
    const Vector6d& x,
    const Matrix6d& P,
    const Vector6d& imuVec,
    const Vector6d* gpsVec = nullptr,
    const std::string& path = "kf_log.csv"
) {
    std::lock_guard<std::mutex> lock(g_kfCsvMutex);

    std::ostringstream ss;
    ss << std::setprecision(17);

    double nowMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();

    ss << g_kfCsvStep++ << "," << nowMs;

    for (int i = 0; i < 6; i++) {
        ss << "," << x(i);
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ss << "," << P(i, j);
        }
    }

    for (int i = 0; i < 6; i++) {
        ss << "," << imuVec(i);
    }

    ss << "," << (gpsVec != nullptr ? 1 : 0);

    for (int i = 0; i < 6; i++) {
        if (gpsVec != nullptr) {
            ss << "," << (*gpsVec)(i);
        } else {
            ss << ",";
        }
    }

    g_kfCsvRows.push_back(ss.str());

    if (g_kfCsvRows.size() < 20) {
        return;
    }

    if (g_kfCsvWriter.valid() && g_kfCsvWriter.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }

    std::vector<std::string> rows;
    rows.swap(g_kfCsvRows);
    bool writeHeader = !g_kfCsvHeaderWritten;
    g_kfCsvHeaderWritten = true;

    g_kfCsvWriter = std::async(std::launch::async, [rows = std::move(rows), path, writeHeader]() {
        std::ofstream out(path, std::ios::app);

        if (writeHeader) {
            out << "step,time_ms";
            for (int i = 0; i < 6; i++) out << ",x" << i;
            for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) out << ",P" << i << j;
            for (int i = 0; i < 6; i++) out << ",imu" << i;
            out << ",has_gps";
            for (int i = 0; i < 6; i++) out << ",gps" << i;
            out << "\n";
        }

        for (const auto& row : rows) {
            out << row << "\n";
        }
    });
}