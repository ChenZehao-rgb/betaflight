/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>

#include "gtest/gtest.h"
#include "unittest_macros.h"

extern "C" {
#include "sitl_gyro.h"
}

TEST(SitlGyroUnittest, SensorFrdToBodyFluAllSignedAxes) {
	for (int axis = 0; axis < 3; ++axis) {
		for (double sign : {-1.0, 1.0}) {
			double sensor[3] = {0, 0, 0};
			// An Rx(pi) sensor mount sees body +Y/+Z as sensor -Y/-Z.
			sensor[axis] = axis == 0 ? sign : -sign;
			double body[3];
			sitlGyroBodyFromSim(sensor, &body[0], &body[1], &body[2]);
			for (int i = 0; i < 3; ++i) {
				EXPECT_DOUBLE_EQ(body[i], i == axis ? sign : 0.0);
			}
		}
	}
}

static void eulerQuaternion(double roll, double pitch, double yaw, double q[4]) {
	const double cr = cos(roll / 2), sr = sin(roll / 2);
	const double cp = cos(pitch / 2), sp = sin(pitch / 2);
	const double cy = cos(yaw / 2), sy = sin(yaw / 2);
	q[0] = cr * cp * cy + sr * sp * sy;
	q[1] = sr * cp * cy - cr * sp * sy;
	q[2] = cr * sp * cy + sr * cp * sy;
	q[3] = cr * cp * sy - sr * sp * cy;
}

TEST(SitlGyroUnittest, GazeboAttitudeHasCorrectWorldAndBodyFrames) {
	// Compass headings, wrap boundary, and tilted attitudes: a yaw-only test
	// cannot detect an incorrect body-frame conjugation.
	for (double yaw : {-M_PI, -M_PI / 2, -0.3, 0.0, 0.3, M_PI / 2, M_PI}) {
		for (double roll : {-0.4, 0.0, 0.4}) {
			for (double pitch : {-0.2, 0.0, 0.2}) {
				double world[4], expected[4];
				eulerQuaternion(roll, pitch, yaw, world);
				eulerQuaternion(roll, pitch, yaw - M_PI / 2, expected);
				const double packet[4] = {world[0], world[1], -world[2], -world[3]};
				float actual[4];
				sitlAttitudeFromGazebo(packet, &actual[0], &actual[1], &actual[2], &actual[3]);
				for (int i = 0; i < 4; ++i) {
					EXPECT_NEAR(actual[i], expected[i], 1e-7);
				}
			}
		}
	}
}
