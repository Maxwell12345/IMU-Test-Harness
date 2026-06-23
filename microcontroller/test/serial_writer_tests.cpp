/******************************************************************************
 * File:             serial_writer_tests.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/23/26 12:49 PM
 *
 ******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>

namespace {
#pragma pack(push, 1)
    typedef struct packed_acceleration_t {
        float x;
        float y;
        float z;
        uint64_t timestamp;
    } packed_acceleration_t;

    typedef struct packed_rotation_t {
        float i;
        float j;
        float k;
        float real;
        float accuracy;
        uint64_t timestamp;
    } packed_rotation_t;
#pragma pack(pop)

    typedef struct acceleration_t {
        float x;
        float y;
        float z;
        uint64_t timestamp;
    } acceleration_t;

    typedef struct rotation_t {
        float i;
        float j;
        float k;
        float real;
        float accuracy;
        uint64_t timestamp;
    } rotation_t;
}

TEST(SerialWriter, TestFieldWiseMemcpy) {
    acceleration_t acc1 = {1.0f, 2.0f, 3.0f, 64u};

    unsigned char buffer[20] = {0};
    std::memcpy(buffer, &acc1.x, sizeof(acc1.x));
    std::memcpy(buffer + 4, &acc1.y, sizeof(acc1.y));
    std::memcpy(buffer + 8, &acc1.z, sizeof(acc1.z));
    std::memcpy(buffer + 12, &acc1.timestamp, sizeof(acc1.timestamp));

    float xCopy;
    float yCopy;
    float zCopy;
    uint64_t timestampCopy;

    std::memcpy(&xCopy, buffer, sizeof(xCopy));
    std::memcpy(&yCopy, buffer + 4, sizeof(yCopy));
    std::memcpy(&zCopy, buffer + 8, sizeof(zCopy));
    std::memcpy(&timestampCopy, buffer + 12, sizeof(timestampCopy));

    ASSERT_FLOAT_EQ(xCopy, acc1.x);
    ASSERT_FLOAT_EQ(yCopy, acc1.y);
    ASSERT_FLOAT_EQ(zCopy, acc1.z);
    ASSERT_EQ(timestampCopy, acc1.timestamp);
}

TEST(SerialWriter, TestUnpackedAccelStructMemcpy) {
    acceleration_t acc1 = {1.0f, 2.0f, 3.0f, 64u};
    unsigned char buffer[20] = {0};

    std::memcpy(buffer, &acc1, sizeof(acceleration_t));

    ASSERT_EQ(sizeof(acceleration_t), (size_t)(4+3*sizeof(float)+sizeof(uint64_t)));

    acceleration_t acc2 = {-1.0f, -1.0f, -1.0f, 0u};
    std::memcpy(&acc2, buffer, sizeof(acceleration_t));

    ASSERT_FLOAT_EQ(acc1.x, acc2.x);
    ASSERT_FLOAT_EQ(acc1.y, acc2.y);
    ASSERT_FLOAT_EQ(acc1.z, acc2.z);
    ASSERT_EQ(acc1.timestamp, acc2.timestamp);
}

TEST(SerialWriter, TestPackedAccelStructMemcpy) {
    packed_acceleration_t acc1 = {1.0f, 2.0f, 3.0f, 64u};
    unsigned char buffer[20] = {0};

    std::memcpy(buffer, &acc1, sizeof(packed_acceleration_t));

    ASSERT_EQ(sizeof(packed_acceleration_t), (size_t)(3*sizeof(float)+sizeof(uint64_t)));

    packed_acceleration_t acc2 = {-1.0f, -1.0f, -1.0f, 0u};
    std::memcpy(&acc2, buffer, sizeof(acceleration_t));

    ASSERT_FLOAT_EQ(acc1.x, acc2.x);
    ASSERT_FLOAT_EQ(acc1.y, acc2.y);
    ASSERT_FLOAT_EQ(acc1.z, acc2.z);
    ASSERT_EQ(acc1.timestamp, acc2.timestamp);
}
