#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "SerialComService.hpp"
#include "SerialPortBase.hpp"

class MockSerialPort : public SerialPortBase {
};

TEST(SerialComServiceTest, SerialComServiceConstructorThrowsRuntimeError) {
    std::function f = [](SerialPortBase& serial){};
    EXPECT_THROW(SerialComService("invalid path", 9600, std::make_unique<MockSerialPort>()), std::invalid_argument);
}

TEST(SerialComServiceTest, SerialComServiceConstructorReturns) {
    EXPECT_NO_THROW(SerialComService("/dev/ttyUSB0", 9600, std::make_unique<MockSerialPort>()));
}

TEST(SerialComServiceTest, InitialValues) {
    std::string path = "/dev/ttyUSB0";
    unsigned int baudRate = 9600;
    SerialComService SerialComService(path, baudRate, std::make_unique<MockSerialPort>());
    EXPECT_FALSE(SerialComService.m_running);
    EXPECT_EQ(SerialComService.m_path, path);
    EXPECT_EQ(SerialComService.m_baudRate, baudRate);
    EXPECT_NE(SerialComService.m_serial, nullptr);
}

TEST(SerialComServiceTest, ServiceLoopStatus) {
    std::string path = "/dev/ttyUSB0";
    unsigned int baudRate = 9600;
    SerialComService SerialComService(path, baudRate, std::make_unique<MockSerialPort>());
    SerialComService.Start();
    EXPECT_TRUE(SerialComService.m_running);
    SerialComService.Stop();
    EXPECT_FALSE(SerialComService.m_running);
}

TEST(SerialComServiceTest, VerifyPathReturnsTrue) {
    EXPECT_TRUE(SerialComService::VerifyPath("/dev/serial/by-id/port0"));
    EXPECT_TRUE(SerialComService::VerifyPath("/dev/serial/by-path/port0"));
    EXPECT_TRUE(SerialComService::VerifyPath("/dev/tty"));
    EXPECT_TRUE(SerialComService::VerifyPath("/dev/tty0"));
    EXPECT_TRUE(SerialComService::VerifyPath("/dev/ttyUSB0"));
}

TEST(SerialComServiceTest, VerifyPathReturnsFalse) {
    EXPECT_FALSE(SerialComService::VerifyPath("/d"));
    EXPECT_FALSE(SerialComService::VerifyPath("/do"));
    EXPECT_FALSE(SerialComService::VerifyPath("/dev/sdf fsdf"));
    EXPECT_FALSE(SerialComService::VerifyPath("/dev/ /"));
    EXPECT_FALSE(SerialComService::VerifyPath("/dev/ "));
}
