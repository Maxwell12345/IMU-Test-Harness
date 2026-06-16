#include <memory>

#include <gtest/gtest.h>

#include "GpsService.hpp"
#include "SerialPort/BoostSerialPort.hpp"
#include "MockClasses/MockSerialPort.hpp"

TEST(GpsServiceTest, GpsService_Constructor_Throws_RuntimeError) {
    EXPECT_THROW(GpsService("invalid path", 9600, std::make_unique<BoostSerialPort>()), std::runtime_error);
}

TEST(GpsServiceTest, GpsService_Constructor_Returns) {
    EXPECT_NO_THROW(GpsService("doesnt matter path", 9600, std::make_unique<MockSerialPort>()));
}

TEST(GpsServiceTest, Initial_Values) {
    std::string path = "THIS IS A VALID PATH";
    unsigned int baudRate = 9600;
    GpsService gpsService(path, baudRate, std::make_unique<MockSerialPort>());
    EXPECT_EQ(gpsService.m_running, false);
    EXPECT_EQ(gpsService.m_path, path);
    EXPECT_EQ(gpsService.m_baudRate, baudRate);
    EXPECT_NE(gpsService.m_serial, nullptr);
}

TEST(GpsServiceTest, Service_Loop_Status) {
    std::string path = "THIS IS A VALID PATH";
    unsigned int baudRate = 9600;
    GpsService gpsService(path, baudRate, std::make_unique<MockSerialPort>());
    gpsService.Start();
    EXPECT_EQ(gpsService.m_running, true);
    gpsService.Stop();
    EXPECT_EQ(gpsService.m_running, false);
}
