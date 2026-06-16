#include <gtest/gtest.h>

#include "GpsService.hpp"

TEST(GpsServiceTest, GpsService_Constructor_Throws_RuntimeError) {
    EXPECT_THROW(GpsService("invalid path", 9600), std::runtime_error);
}
