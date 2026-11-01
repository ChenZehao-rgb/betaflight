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

#pragma once

// Gazebo's IMU mount is Rx(pi): sensor FRD -> Betaflight body FLU.
// Gyro and Mahony quaternion integration use the same right-handed body frame;
// the clockwise-positive RC yaw convention is handled by fc/rc.c, not here.
// Legacy bridges already use this (x, -y, -z) conversion.
static inline void sitlGyroBodyFromSim(const double rpy[3],
                                     double *roll, double *pitch, double *yaw)
{
    *roll = rpy[0];
    *pitch = -rpy[1];
    *yaw = -rpy[2];
}

// The Gazebo plugin sends Rx(pi) * q_FLU_to_ENU * Rx(pi)^-1.
// Undo that conjugation, then change world coordinates ENU -> NWU with Rz(-pi/2).
static inline void sitlAttitudeFromGazebo(const double packet[4],
                                         float *w, float *x, float *y, float *z)
{
    const double k = 0.7071067811865475244;
    *w = k * (packet[0] - packet[3]);
    *x = k * (packet[1] - packet[2]);
    *y = k * (-packet[2] - packet[1]);
    *z = k * (-packet[3] - packet[0]);
}
