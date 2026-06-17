#include "Utils.hpp"

#include <iostream>

void utils::LOG_DEBUG(std::string str, bool flush) {
    std::cout << "[DEBUG] " << str;

    if (flush) {
        std::cout.flush();
    }
    else {
        std::cout << '\n';
    }
}

void utils::LOG_INFO(std::string str, bool flush) {
    std::cout << "[INFO] " << str;

    if (flush) {
        std::cout.flush();
    }
    else {
        std::cout << '\n';
    }
}

void utils::LOG_WARN(std::string str) {
    std::cout << "[WARN] " << str << std::endl;
}

void utils::LOG_ERROR(std::string str) {
    std::cout << "[ERROR] " << str << std::endl;
}

