/******************************************************************************
 * File:             SimulationReplayTests.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Verifies reuse of recorded NMEA sentences and database
 *                   timestamps without constructing live serial hardware.
 *
 ******************************************************************************/

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "GpsUpdate.hpp"
#include "gps/GpsManager.hpp"
#include "gps/NmeaReader.hpp"

TEST(SimulationReplayTest, RecordedRmcMapsDatabaseTimestampIntoGpsUpdate) {
    const NmeaMessage message = NmeaReader::Parse(
        "$GNRMC,150729.00,A,3241.81589,N,11713.97397,W,17.870,165.40,050826,,,D,V*2C");
    const GpsUpdate update = GpsManager::BuildGpsUpdate(message, 1'060'413'146'719ULL);

    EXPECT_TRUE(update.valid);
    EXPECT_NEAR(update.latitude, 32.6969315, 1e-7);
    EXPECT_NEAR(update.longitude, -117.2328995, 1e-7);
    EXPECT_NEAR(update.timestamp, 1060.413146719, 1e-9);
    EXPECT_EQ(update.gpsTimestampMs, 1'060'413U);
    ASSERT_TRUE(update.heading.has_value());
    EXPECT_NEAR(*update.heading, 165.40, 1e-12);
    ASSERT_TRUE(update.measurementYear.has_value());
    EXPECT_EQ(*update.measurementYear, 2026);
}

TEST(SimulationReplayTest, DatabaseTimestampOutsideGpsFieldRangeIsRejected) {
    const NmeaMessage message = NmeaReader::Parse(
        "$GNRMC,150729.00,A,3241.81589,N,11713.97397,W,17.870,165.40,050826,,,D,V*2C");
    const uint64_t overflowTimestampNs =
        (static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) + 1ULL) * 1'000'000ULL;

    EXPECT_THROW(GpsManager::BuildGpsUpdate(message, overflowTimestampNs), std::out_of_range);
}
