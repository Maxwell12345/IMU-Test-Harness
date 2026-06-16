#include "MockSerialPort.hpp"

void MockSerialPort::Open(const std::string& port) {
}

void MockSerialPort::SetBaudRate(unsigned int rate) {
}

std::string MockSerialPort::ReadLine() {
    return "";
}

void MockSerialPort::Close() { 
}