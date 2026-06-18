#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "SerialComService.hpp"
#include "BoostSerialPort.hpp"
#include "MockClasses/MockSerialPort.hpp"

TEST(SerialComServiceTest, SerialComService_Constructor_Throws_RuntimeError) {
    std::function f = [](boost::asio::serial_port& serial){};
    EXPECT_THROW(SerialComService("invalid path", 9600, std::make_unique<BoostSerialPort>(f)), std::runtime_error);
}

TEST(SerialComServiceTest, SerialComService_Constructor_Returns) {
    EXPECT_NO_THROW(SerialComService("doesnt matter path", 9600, std::make_unique<MockSerialPort>()));
}

TEST(SerialComServiceTest, Initial_Values) {
    std::string path = "THIS IS A VALID PATH";
    unsigned int baudRate = 9600;
    SerialComService SerialComService(path, baudRate, std::make_unique<MockSerialPort>());
    EXPECT_EQ(SerialComService.m_running, false);
    EXPECT_EQ(SerialComService.m_path, path);
    EXPECT_EQ(SerialComService.m_baudRate, baudRate);
    EXPECT_NE(SerialComService.m_serial, nullptr);
}

TEST(SerialComServiceTest, Service_Loop_Status) {
    std::string path = "THIS IS A VALID PATH";
    unsigned int baudRate = 9600;
    SerialComService SerialComService(path, baudRate, std::make_unique<MockSerialPort>());
    SerialComService.Start();
    EXPECT_EQ(SerialComService.m_running, true);
    SerialComService.Stop();
    EXPECT_EQ(SerialComService.m_running, false);
}
