#ifndef INU_DISPLAY_USBSERIALLOCATOR_HPP
#define INU_DISPLAY_USBSERIALLOCATOR_HPP

#include <cstdint>
#include <optional>
#include <string>

std::optional<std::string> FindSerialPortByUsbId(uint16_t vendorId, uint16_t productId);

#endif
