#include <gtest/gtest.h>

extern "C" {
#include "utils.h"
}

TEST(ImuMathTest, AddReturnsSum) {
    EXPECT_FLOAT_EQ(Add(2.0f, 3.0f), 5.0f);
}

TEST(ImuMathTest, DegreesToRadians180) {
    EXPECT_NEAR(DegreesToRadians(180.0f), 3.1415926f, 0.0001f);
}

TEST(ImuMathTest, DegreesToRadians90) {
    EXPECT_NEAR(DegreesToRadians(90.0f), 1.5707963f, 0.0001f);
}