// #include <gtest/gtest.h>

// #include "IMUSerialPortReader.hpp"

// #include <pty.h>
// #include <termios.h>
// #include <unistd.h>

// #include <cstring>
// #include <optional>
// #include <stdexcept>
// #include <string>
// #include <vector>

// struct TestPty {
//     int masterFd = -1;
//     int slaveFd = -1;
//     std::string slavePath;

//     TestPty() {
//         char name[128]{};

//         if (openpty(&masterFd, &slaveFd, name, nullptr, nullptr) != 0) {
//             throw std::runtime_error("openpty failed");
//         }

//         termios tio{};
//         tcgetattr(slaveFd, &tio);
//         cfmakeraw(&tio);
//         tcsetattr(slaveFd, TCSANOW, &tio);

//         slavePath = name;
//     }

//     ~TestPty() {
//         if (masterFd >= 0) {
//             close(masterFd);
//         }

//         if (slaveFd >= 0) {
//             close(slaveFd);
//         }
//     }
// };

// template <typename T>
// static std::vector<unsigned char> BuildPacket(unsigned char type, const T& payload) {
//     std::vector<unsigned char> packet;
//     packet.resize(1 + 1 + 1 + sizeof(T) + 2);

//     packet[0] = 0xFF;
//     packet[1] = type;
//     packet[2] = static_cast<unsigned char>(sizeof(T));

//     std::memcpy(packet.data() + 3, &payload, sizeof(T));

//     unsigned int crc = 0xFFFF;

//     for (unsigned long i = 0; i < sizeof(T); ++i) {
//         crc ^= static_cast<unsigned int>((packet.data() + 3)[i]) << 8;

//         for (int j = 0; j < 8; ++j) {
//             if (crc & 0x8000) {
//                 crc = (crc << 1) ^ 0x1021;
//             } else {
//                 crc <<= 1;
//             }

//             crc &= 0xFFFF;
//         }
//     }

//     packet[3 + sizeof(T)] = static_cast<unsigned char>((crc >> 8) & 0xFF);
//     packet[3 + sizeof(T) + 1] = static_cast<unsigned char>(crc & 0xFF);

//     return packet;
// }

// static void WriteAll(int fd, const std::vector<unsigned char>& bytes) {
//     std::size_t written = 0;

//     while (written < bytes.size()) {
//         ssize_t n = write(fd, bytes.data() + written, bytes.size() - written);

//         if (n <= 0) {
//             throw std::runtime_error("write failed");
//         }

//         written += static_cast<std::size_t>(n);
//     }
// }

// TEST(_IMUSerialPortTest, ValidateCalculateCRC16CCITTFalseChecksum) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     unsigned char empty[1] = {0x00};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(empty, 0), 0xFFFFUL);

//     unsigned char a[1] = {'A'};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(a, 1), 0xB915UL);

//     unsigned char zero[1] = {0x00};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(zero, 1), 0xE1F0UL);

//     unsigned char abc[3] = {'a', 'b', 'c'};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(abc, 3), 0x514AUL);

//     unsigned char zeros[4] = {0x00, 0x00, 0x00, 0x00};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(zeros, 4), 0x84C0UL);

//     unsigned char ones[4] = {0xFF, 0xFF, 0xFF, 0xFF};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(ones, 4), 0x1D0FUL);

//     unsigned char check[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(check, 9), 0x29B1UL);

//     unsigned char hello[11] = {'H', 'e', 'l', 'l', 'o', ',', ' ', 'I', 'M', 'U', '!'};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(hello, 11), 0x62BBUL);

//     unsigned char sentence[47] = {'A', ' ', 'c', 'o', 'w', ' ', 'j', 'u', 'm', 'p', 'e', 'd', ' ', 'o', 'v', 'e', 'r', ' ', 't', 'h', 'e', ' ', 'm', 'o', 'o', 'n', ' ', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '!', '@', '#', '$', '%', '*', '(', ')', '_', '+'};
//     EXPECT_EQ(port.CalculateCRC16CCITTFalseChecksum(sentence, 47), 0xD92CUL);
// }

// TEST(_IMUSerialPortTest, ValidateIsStartEncoder) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     unsigned char start = 0xFF;
//     unsigned char zero = 0x00;
//     unsigned char other = 0x7E;

//     EXPECT_TRUE(port.IsStartEncoder(start));
//     EXPECT_FALSE(port.IsStartEncoder(zero));
//     EXPECT_FALSE(port.IsStartEncoder(other));
// }

// TEST(_IMUSerialPortTest, ValidateGetMessageType) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     unsigned char accel = 0x00;
//     unsigned char rot = 0x01;
//     unsigned char bad = 0x02;

//     EXPECT_EQ(port.GetMessageType(accel), _IMU_MESSAGE_TYPES_::ACCELERATION);
//     EXPECT_EQ(port.GetMessageType(rot), _IMU_MESSAGE_TYPES_::ROTATION_VECTOR);
//     EXPECT_THROW(port.GetMessageType(bad), std::runtime_error);
// }

// TEST(_IMUSerialPortTest, ValidateGetMessageLength) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     unsigned char zero = 0x00;
//     unsigned char one = 0x01;
//     unsigned char max = 0xFF;

//     EXPECT_EQ(port.GetMessageLength(zero), 0u);
//     EXPECT_EQ(port.GetMessageLength(one), 1u);
//     EXPECT_EQ(port.GetMessageLength(max), 255u);
// }

// TEST(_IMUSerialPortTest, ValidateValidateMessage) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     unsigned char payload[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
//     unsigned char goodChecksum[2] = {0x29, 0xB1};
//     unsigned char badChecksum[2] = {0x00, 0x00};

//     EXPECT_TRUE(port.ValidateMessage(goodChecksum, payload, 9));
//     EXPECT_FALSE(port.ValidateMessage(badChecksum, payload, 9));
// }

// TEST(_IMUSerialPortTest, ValidateSetBaudRate) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     EXPECT_NO_THROW(port.SetBaudRate(9600u));
//     EXPECT_NO_THROW(port.SetBaudRate(115200u));
// }

// TEST(_IMUSerialPortTest, ValidateOpen) {
//     TestPty pty;

//     EXPECT_NO_THROW({
//         _IMUSerialPort port(pty.slavePath, 115200u);
//     });
// }

// TEST(_IMUSerialPortTest, ValidateClose) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     EXPECT_NO_THROW(port.Close());
//     EXPECT_NO_THROW(port.Close());
// }

// TEST(_IMUSerialPortTest, ValidateConstructor) {
//     TestPty pty;

//     EXPECT_NO_THROW({
//         _IMUSerialPort port(pty.slavePath, 115200u);
//     });
// }

// TEST(_IMUSerialPortTest, CallbackIgnoresNonStartByte) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     bool called = false;

//     port.SetCompletedPayloadCallback(
//         [&](std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>) {
//             called = true;
//         }
//     );

//     std::vector<unsigned char> bytes = {0x7E};
//     WriteAll(pty.masterFd, bytes);

//     EXPECT_NO_THROW(port.Callback());
//     EXPECT_FALSE(called);
// }

// TEST(_IMUSerialPortTest, CallbackReadsAccelerationPacket) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     Raw_Accelerometer expected{};
//     std::memset(&expected, 0x11, sizeof(expected));

//     bool called = false;
//     std::optional<Raw_Accelerometer> receivedAccel;
//     std::optional<Raw_RotationVectorWAcc> receivedRot;

//     port.SetCompletedPayloadCallback(
//         [&](std::optional<Raw_RotationVectorWAcc> rot, std::optional<Raw_Accelerometer> accel) {
//             called = true;
//             receivedRot = rot;
//             receivedAccel = accel;
//         }
//     );

//     auto packet = BuildPacket(0x00, expected);
//     WriteAll(pty.masterFd, packet);

//     EXPECT_NO_THROW(port.Callback());
//     EXPECT_TRUE(called);
//     EXPECT_FALSE(receivedRot.has_value());
//     ASSERT_TRUE(receivedAccel.has_value());
//     EXPECT_EQ(std::memcmp(&expected, &receivedAccel.value(), sizeof(Raw_Accelerometer)), 0);
// }

// TEST(_IMUSerialPortTest, CallbackReadsRotationPacket) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     Raw_RotationVectorWAcc expected{};
//     std::memset(&expected, 0x22, sizeof(expected));

//     bool called = false;
//     std::optional<Raw_Accelerometer> receivedAccel;
//     std::optional<Raw_RotationVectorWAcc> receivedRot;

//     port.SetCompletedPayloadCallback(
//         [&](std::optional<Raw_RotationVectorWAcc> rot, std::optional<Raw_Accelerometer> accel) {
//             called = true;
//             receivedRot = rot;
//             receivedAccel = accel;
//         }
//     );

//     auto packet = BuildPacket(0x01, expected);
//     WriteAll(pty.masterFd, packet);

//     EXPECT_NO_THROW(port.Callback());
//     EXPECT_TRUE(called);
//     EXPECT_FALSE(receivedAccel.has_value());
//     ASSERT_TRUE(receivedRot.has_value());
//     EXPECT_EQ(std::memcmp(&expected, &receivedRot.value(), sizeof(Raw_RotationVectorWAcc)), 0);
// }

// TEST(_IMUSerialPortTest, CallbackRejectsBadChecksum) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     Raw_Accelerometer payload{};
//     std::memset(&payload, 0x33, sizeof(payload));

//     bool called = false;

//     port.SetCompletedPayloadCallback(
//         [&](std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>) {
//             called = true;
//         }
//     );

//     auto packet = BuildPacket(0x00, payload);
//     packet[packet.size() - 1] ^= 0xFF;

//     WriteAll(pty.masterFd, packet);

//     EXPECT_NO_THROW(port.Callback());
//     EXPECT_FALSE(called);
// }

// TEST(_IMUSerialPortTest, CallbackThrowsOnBadMessageType) {
//     TestPty pty;
//     _IMUSerialPort port(pty.slavePath, 115200u);

//     std::vector<unsigned char> packet = {0xFF, 0x02};

//     WriteAll(pty.masterFd, packet);

//     EXPECT_THROW(port.Callback(), std::runtime_error);
// }