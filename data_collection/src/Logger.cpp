#include "Logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

std::atomic<LogLevel> Logger::s_level{LogLevel::Info};

namespace {

std::mutex g_outputMutex;

const char* levelName(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO ";
        case LogLevel::Warn:  return "WARN ";
        case LogLevel::Error: return "ERROR";
    }
    return "?????";
}

std::string timestamp() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto seconds = system_clock::to_time_t(now);
    const auto millis = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm local{};
    localtime_r(&seconds, &local);

    std::ostringstream out;
    out << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3)
        << std::setfill('0') << millis.count();
    return out.str();
}

}  // namespace

void Logger::setLevel(LogLevel level) {
    s_level.store(level, std::memory_order_relaxed);
}

LogLevel Logger::level() {
    return s_level.load(std::memory_order_relaxed);
}

bool Logger::enabled(LogLevel level) {
    return static_cast<int>(level) >= static_cast<int>(s_level.load(std::memory_order_relaxed));
}

void Logger::write(LogLevel level, std::string_view component, std::string_view message) {
    std::lock_guard<std::mutex> lock(g_outputMutex);
    std::cerr << timestamp() << " [" << levelName(level) << "] " << component << ": "
              << message << "\n";
}

LogMessage::LogMessage(LogLevel level, std::string component)
    : m_level(level), m_component(std::move(component)) {}

LogMessage::~LogMessage() {
    Logger::write(m_level, m_component, m_stream.str());
}
