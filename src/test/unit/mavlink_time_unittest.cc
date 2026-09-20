#include <gtest/gtest.h>
extern "C" {
#include "telemetry/mavlink_time.h"
}

TEST(MavlinkTime, ExtendsMicrosAcrossRepeatedWraps)
{
    mavlinkClock_t clock = {};
    EXPECT_EQ(mavlinkClockUpdate(&clock, 100), 100u);
    EXPECT_EQ(mavlinkClockUpdate(&clock, UINT32_MAX - 4), UINT32_MAX - 4ULL);
    EXPECT_EQ(mavlinkClockUpdate(&clock, 5), (1ULL << 32) + 5);
    EXPECT_EQ(mavlinkClockUpdate(&clock, 5), (1ULL << 32) + 5);
    EXPECT_EQ(mavlinkClockUpdate(&clock, UINT32_MAX), (2ULL << 32) - 1);
    EXPECT_EQ(mavlinkClockUpdate(&clock, 8), (2ULL << 32) + 8);
}
