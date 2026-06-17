#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>

namespace utils {
    void LOG_DEBUG(std::string str, bool flush = false);
    void LOG_INFO(std::string str, bool flush = false);
    void LOG_WARN(std::string str);
    void LOG_ERROR(std::string str);
}

#endif