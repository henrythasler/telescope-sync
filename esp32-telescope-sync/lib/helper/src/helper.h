#ifndef HELPER_H
#define HELPER_H

#include <stdint.h>
#include <ctime>
#include <math.h>

namespace MathHelper
{
    double f_mod(double a, double n);
    double getLocalSiderealTimeDegrees(tm utcTimestamp, double longitude);
    float julianDay(tm timestamp);
}

namespace Checksum
{
    bool verifyAmtCheckbits(uint32_t data);
}
#endif