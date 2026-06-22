/******************************************************************************
 * File:             serial_tests.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/22/26 8:01 AM
 *
 ******************************************************************************/
#include <gtest/gtest.h>

#include "../main/serial.h"
#include <chrono>

extern "C" {
#include "serial.h"
}

TEST(serial, Write_Accel_Msg) {
    uint64_t t= 0x445566778899AABB;
    acceleration_t acceleration = {
        1.0f,
        2.0f,
        3.0f,
        t
    };

    ASSERT_NO_THROW(send_acceleration_t(&acceleration));
}