#include <gtest/gtest.h>

#include "IMUSerialPortReader.hpp"

TEST(IMUSerialPortReaderTest, ValidateCalculateCRC16CCITTFalseChecksum) {
    IMUSerialPortReader reader("/dev/ttyUSB0", 115200u, nullptr);

    unsigned char empty[1] = {0x00};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(empty, 0), 0xFFFFUL);

    unsigned char a[1] = {'A'};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(a, 1), 0xB915UL);

    unsigned char zero[1] = {0x00};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(zero, 1), 0xE1F0UL);

    unsigned char abc[3] = {'a', 'b', 'c'};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(abc, 3), 0x514AUL);

    unsigned char zeros[4] = {0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(zeros, 4), 0x84C0UL);

    unsigned char ones[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(ones, 4), 0x1D0FUL);

    unsigned char check[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(check, 9), 0x29B1UL);

    unsigned char hello[11] = {'H', 'e', 'l', 'l', 'o', ',', ' ', 'I', 'M', 'U', '!'};
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(hello, 11), 0x62BBUL);

    unsigned char sentence[47] = {'A', ' ', 'c', 'o', 'w', ' ', 'j', 'u', 'm', 'p', 'e', 'd', ' ', 'o', 'v', 'e', 'r', ' ', 't', 'h', 'e', ' ', 'm', 'o', 'o', 'n', ' ', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '!', '@', '#', '$', '%', '*', '(', ')', '_', '+'};

    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(sentence, 47), 0xD92C);
    
}
