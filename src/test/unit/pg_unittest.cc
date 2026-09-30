/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>

extern "C" {
#include <platform.h>

#include "build/debug.h"
#include "flight/mixer.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

//PG_DECLARE(motorConfig_t, motorConfig);

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);
PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 4);
PG_RESET_TEMPLATE(rxConfig_t, rxConfig, .msp_override_timeout_ms = 300);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .dev = {
        .motorPwmRate = 400,
        .motorProtocol = 0,
        .motorInversion = 0,
        .useContinuousUpdate = 0,
        .useBurstDshot = 0,
        .useDshotTelemetry = 0,
        .useDshotEdt = 0,
        .ioTags = {IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE},
        .motorTransportProtocol = 0,
        .useDshotBitbang = 0,
        .useDshotBitbangedTimer = 0,
        .motorOutputReordering = {0, 1, 2, 3},
    },
    .motorIdle = 0,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .kv = 1000,
    .motorPoleCount = 14,
);
}


#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(ParameterGroupsfTest, Test_pgResetAll)
{
    memset(motorConfigMutable(), 0, sizeof(motorConfig_t));
    pgResetAll();
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);
}

TEST(ParameterGroupsfTest, Test_pgFind)
{
    memset(motorConfigMutable(), 0, sizeof(motorConfig_t));
    const pgRegistry_t *pgRegistry = pgFind(PG_MOTOR_CONFIG);
    pgReset(pgRegistry);
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);

    motorConfig_t motorConfig2;
    memset(&motorConfig2, 0, sizeof(motorConfig_t));
    motorConfigMutable()->dev.motorPwmRate = 500;
    pgStore(pgRegistry, &motorConfig2, sizeof(motorConfig_t));
    EXPECT_EQ(1850, motorConfig2.maxthrottle);
    EXPECT_EQ(1000, motorConfig2.mincommand);
    EXPECT_EQ(500, motorConfig2.dev.motorPwmRate);

    motorConfig_t motorConfig3;
    memset(&motorConfig3, 0, sizeof(motorConfig_t));
    pgResetCopy(&motorConfig3, PG_MOTOR_CONFIG);
    EXPECT_EQ(1850, motorConfig3.maxthrottle);
    EXPECT_EQ(1000, motorConfig3.mincommand);
    EXPECT_EQ(400, motorConfig3.dev.motorPwmRate);
}

// STUBS

extern "C" {
}

TEST(ParameterGroupsfTest, ReceiverVersionFourExtensionPreservesLegacyBytes) {
	static_assert(offsetof(rxConfig_t, msp_override_timeout_ms) == 52, "preserve RX v4 prefix including padding");
	static_assert(sizeof(rxConfig_t) == 56, "RX extension layout");
	const pgRegistry_t* registry = pgFind(PG_RX_CONFIG);
	uint8_t legacy[52];
	memset(legacy, 0xa5, sizeof(legacy));
	EXPECT_TRUE(pgLoad(registry, legacy, sizeof(legacy), 4));
	EXPECT_EQ(0, memcmp(rxConfig(), legacy, sizeof(legacy)));
	EXPECT_EQ(300u, rxConfig()->msp_override_timeout_ms);
	rxConfigMutable()->msp_override_timeout_ms = 50;
	rxConfig_t stored;
	EXPECT_EQ(sizeof(stored), static_cast<size_t>(pgStore(registry, &stored, sizeof(stored))));
	pgReset(registry);
	EXPECT_TRUE(pgLoad(registry, &stored, sizeof(stored), 4));
	EXPECT_EQ(50u, rxConfig()->msp_override_timeout_ms);
	EXPECT_EQ(0, memcmp(rxConfig(), legacy, sizeof(legacy)));
}
