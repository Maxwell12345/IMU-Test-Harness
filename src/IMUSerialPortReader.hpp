#ifndef IMU_SERIAL_PORT_READER_HPP
#define IMU_SERIAL_PORT_READER_HPP

#include <gtest/gtest_prod.h> 
#include "SerialComService.hpp"
#include "SerialPort/SerialPortBase.hpp"
#include "imu_data.hpp"
#include "crc/crc.h"

class IMUSerialPortReader {
public:
    IMUSerialPortReader(std::string path,
                        unsigned int baudRate,
                        std::unique_ptr<SerialPortBase> serialPort);

    void InstallVectorCallback();

private:
    unsigned long CalculateCRC16CCITTFalseChecksum(unsigned char* payload, unsigned long len);

private:
    SerialComService m_serialComService;
    cm_t m_cm;

    FRIEND_TEST(IMUSerialPortReaderTest, ValidateCalculateCRC16CCITTFalseChecksum);
};

#endif // IMU_SERIAL_PORT_READER_HPP