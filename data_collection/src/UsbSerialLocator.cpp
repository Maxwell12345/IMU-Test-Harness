#include "UsbSerialLocator.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

std::optional<uint16_t> readHexId(const fs::path& file) {
    std::ifstream input(file);
    if (!input.is_open()) {
        return std::nullopt;
    }

    std::string text;
    std::getline(input, text);
    if (text.empty()) {
        return std::nullopt;
    }

    try {
        return static_cast<uint16_t>(std::stoul(text, nullptr, 16));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace

std::optional<std::string> FindSerialPortByUsbId(uint16_t vendorId, uint16_t productId) {
    const fs::path ttyRoot{"/sys/class/tty"};
    std::error_code ec;
    if (!fs::exists(ttyRoot, ec)) {
        return std::nullopt;
    }

    for (const auto& entry : fs::directory_iterator(ttyRoot, ec)) {
        const fs::path deviceLink = entry.path() / "device";
        if (!fs::exists(deviceLink, ec)) {
            continue;
        }

        fs::path current = fs::canonical(deviceLink, ec);
        if (ec) {
            continue;
        }

        while (!current.empty() && current != current.root_path()) {
            const fs::path vendorFile = current / "idVendor";
            const fs::path productFile = current / "idProduct";

            if (fs::exists(vendorFile, ec) && fs::exists(productFile, ec)) {
                if (readHexId(vendorFile) == vendorId && readHexId(productFile) == productId) {
                    return "/dev/" + entry.path().filename().string();
                }
                break;
            }
            current = current.parent_path();
        }
    }

    return std::nullopt;
}
