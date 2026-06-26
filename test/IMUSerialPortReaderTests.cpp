#include <gtest/gtest.h>

#include "IMUSerialPortReader.hpp"

#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>

#include <YamlConfig.hpp>

namespace {
#ifdef _WIN32
    std::string path = R"(\\.\COM10)";
#else
    std::string path = R"(/dev/ttyUSB0)";
#endif
}

namespace {
    _ImuSerialPort config = {
        path,
        460800
    };
}

class MockSerialPort : public SerialPortBase {
public:
    void ReadExact(unsigned char* data, std::size_t len) override {
        if (len > m_data.size()) {
            return;
        }

        std::memcpy(data, m_data.data(), len);

        m_data.erase(m_data.begin(), m_data.begin() + len);
    }

    void InstallCallback(std::function<void(SerialPortBase& port)> callback) {
        m_callback = callback;
    }

    void Callback() {
        m_callback(*this);
    }

    std::vector<unsigned char> m_data;
    std::function<void(SerialPortBase&)> m_callback;
};

template <typename T>
static std::vector<unsigned char> BuildPacket(unsigned char type, const T& payload) {
    std::vector<unsigned char> packet;
    packet.resize(1 + 1 + 1 + sizeof(T) + 2);

    packet[0] = 0xFF;
    packet[1] = type;
    packet[2] = static_cast<unsigned char>(sizeof(T));

    std::memcpy(packet.data() + 3, &payload, sizeof(T));

    unsigned int crc = 0xFFFF;

    for (unsigned long i = 0; i < sizeof(T); ++i) {
        crc ^= static_cast<unsigned int>((packet.data() + 3)[i]) << 8;

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }

            crc &= 0xFFFF;
        }
    }

    packet[3 + sizeof(T)] = static_cast<unsigned char>((crc >> 8) & 0xFF);
    packet[3 + sizeof(T) + 1] = static_cast<unsigned char>(crc & 0xFF);

    return packet;
}

TEST(_IMUSerialPortTest, ValidateCalculateCRC16CCITTFalseChecksum) {
    IMUSerialPortReader reader(config, std::make_unique<MockSerialPort>());

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
    EXPECT_EQ(reader.CalculateCRC16CCITTFalseChecksum(sentence, 47), 0xD92CUL);
}

TEST(_IMUSerialPortTest, ValidateIsStartEncoder) {
    IMUSerialPortReader reader(config, std::make_unique<MockSerialPort>());

    unsigned char start = 0xFF;
    unsigned char zero = 0x00;
    unsigned char other = 0x7E;

    EXPECT_TRUE(reader.IsStartEncoder(start));
    EXPECT_FALSE(reader.IsStartEncoder(zero));
    EXPECT_FALSE(reader.IsStartEncoder(other));
}

TEST(_IMUSerialPortTest, ValidateGetMessageType) {
    IMUSerialPortReader reader(config, std::make_unique<MockSerialPort>());

    unsigned char accel = 0x00;
    unsigned char rot = 0x01;
    unsigned char bad = 0x02;

    EXPECT_EQ(reader.GetMessageType(accel), _IMU_MESSAGE_TYPES_::ACCELERATION);
    EXPECT_EQ(reader.GetMessageType(rot), _IMU_MESSAGE_TYPES_::ROTATION_VECTOR);
    EXPECT_THROW(reader.GetMessageType(bad), std::runtime_error);
}

TEST(_IMUSerialPortTest, ValidateGetMessageLength) {
    IMUSerialPortReader reader(config, std::make_unique<MockSerialPort>());

    unsigned char zero = 0x00;
    unsigned char one = 0x01;
    unsigned char max = 0xFF;

    EXPECT_EQ(reader.GetMessageLength(zero), 0u);
    EXPECT_EQ(reader.GetMessageLength(one), 1u);
    EXPECT_EQ(reader.GetMessageLength(max), 255u);
}

TEST(_IMUSerialPortTest, ValidateValidateMessage) {
    IMUSerialPortReader reader(config, std::make_unique<MockSerialPort>());

    unsigned char payload[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    unsigned char goodChecksum[2] = {0x29, 0xB1};
    unsigned char badChecksum[2] = {0x00, 0x00};

    EXPECT_TRUE(reader.ValidateMessage(goodChecksum, payload, 9));
    EXPECT_FALSE(reader.ValidateMessage(badChecksum, payload, 9));
}

TEST(_IMUSerialPortTest, CallbackIgnoresNonStartByte) {
    bool called = false;
    std::vector<unsigned char> bytes = {0x7E};

    auto port = std::make_unique<MockSerialPort>();
    port->m_data = bytes;

    IMUSerialPortReader reader(config, std::move(port));
    reader.InstallCallback(
        [&](std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>) {
            called = true;
        }
    );

    reader.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reader.Stop();

    EXPECT_FALSE(called);
}

TEST(_IMUSerialPortTest, CallbackReadsAccelerationPacket) {
    Raw_Accelerometer expected{};
    std::memset(&expected, 0x11, sizeof(expected));
    auto packet = BuildPacket(0x00, expected);
    
    auto port = std::make_unique<MockSerialPort>();
    port->m_data = packet;

    IMUSerialPortReader reader(config, std::move(port));

    bool called = false;
    std::optional<Raw_Accelerometer> receivedAccel;
    std::optional<Raw_RotationVectorWAcc> receivedRot;

    reader.InstallCallback(
        [&](std::optional<Raw_RotationVectorWAcc> rot, std::optional<Raw_Accelerometer> accel) {
            called = true;
            receivedRot = rot;
            receivedAccel = accel;
        }
    );

    reader.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reader.Stop();

    EXPECT_TRUE(called);
    EXPECT_FALSE(receivedRot.has_value());
    ASSERT_TRUE(receivedAccel.has_value());
    EXPECT_EQ(std::memcmp(&expected, &receivedAccel.value(), sizeof(Raw_Accelerometer)), 0);
}

TEST(_IMUSerialPortTest, CallbackReadsRotationPacket) {
    Raw_RotationVectorWAcc expected{};
    std::memset(&expected, 0x22, sizeof(expected));
    auto packet = BuildPacket(0x01, expected);
    
    auto port = std::make_unique<MockSerialPort>();
    port->m_data = packet;
    IMUSerialPortReader reader(config, std::move(port));

    bool called = false;
    std::optional<Raw_Accelerometer> receivedAccel;
    std::optional<Raw_RotationVectorWAcc> receivedRot;

    reader.InstallCallback(
        [&](std::optional<Raw_RotationVectorWAcc> rot, std::optional<Raw_Accelerometer> accel) {
            called = true;
            receivedRot = rot;
            receivedAccel = accel;
        }
    );

    reader.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reader.Stop();

    EXPECT_TRUE(called);
    EXPECT_FALSE(receivedAccel.has_value());
    ASSERT_TRUE(receivedRot.has_value());
    EXPECT_EQ(std::memcmp(&expected, &receivedRot.value(), sizeof(Raw_RotationVectorWAcc)), 0);
}

TEST(_IMUSerialPortTest, CallbackRejectsBadChecksum) {
    Raw_Accelerometer payload{};
    std::memset(&payload, 0x33, sizeof(payload));
    auto packet = BuildPacket(0x00, payload);
    packet[packet.size() - 1] ^= 0xFF;

    auto port = std::make_unique<MockSerialPort>();
    port->m_data = packet;
    IMUSerialPortReader reader(config, std::move(port));

    bool called = false;

    reader.InstallCallback(
        [&](std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>) {
            called = true;
        }
    );

    reader.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reader.Stop();

    EXPECT_FALSE(called);
}

TEST(_IMUSerialPortTest, CallbackThrowsOnBadMessageType) {
    
    std::vector<unsigned char> packet = {0xFF, 0x02};

    auto port = std::make_unique<MockSerialPort>();
    port->m_data = packet;
    IMUSerialPortReader reader(config, std::move(port));

    port = std::make_unique<MockSerialPort>();
    port->m_data = packet;
    EXPECT_THROW(reader.Callback(*port), std::runtime_error);
}
