#ifndef INU_DISPLAY_LOGGER_HPP
#define INU_DISPLAY_LOGGER_HPP

#include <atomic>
#include <sstream>
#include <string>
#include <string_view>

enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };

class Logger {
public:
    static void setLevel(LogLevel level);
    static LogLevel level();
    static bool enabled(LogLevel level);
    static void write(LogLevel level, std::string_view component, std::string_view message);

private:
    static std::atomic<LogLevel> s_level;
};

class LogMessage {
public:
    LogMessage(LogLevel level, std::string component);
    ~LogMessage();

    LogMessage(const LogMessage&) = delete;
    LogMessage& operator=(const LogMessage&) = delete;

    template <typename T>
    LogMessage& operator<<(const T& value) {
        m_stream << value;
        return *this;
    }

private:
    LogLevel m_level;
    std::string m_component;
    std::ostringstream m_stream;
};

#define INU_LOG(level, component) \
    if (!::Logger::enabled(level)) { \
    } else \
        ::LogMessage((level), (component))

#define LOG_DEBUG(component) INU_LOG(LogLevel::Debug, component)
#define LOG_INFO(component) INU_LOG(LogLevel::Info, component)
#define LOG_WARN(component) INU_LOG(LogLevel::Warn, component)
#define LOG_ERROR(component) INU_LOG(LogLevel::Error, component)

#endif
