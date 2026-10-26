/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Unlike isfinite(), this check remains meaningful under the firmware's
// -ffast-math / -Ofast flags. All supported targets use IEEE-754 binary32.
static inline bool float32IsFinite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}
