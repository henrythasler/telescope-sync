#ifndef SIDEREALTIME_H
#define SIDEREALTIME_H

using namespace std;

#include <stdint.h>
#include <ctime>
#include <math.h>

namespace MathHelper
{
    double f_mod(double a, double n);
    double getLocalSiderealTimeDegrees(tm utcTimestamp, double longitude);
    float julianDay(tm timestamp);
}
#endif