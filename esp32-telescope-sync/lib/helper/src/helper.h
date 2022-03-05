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

namespace Checksum
{
    bool verifyAmtCheckbits(uint32_t data);
}

namespace LinAlg
{
    struct Point
    {
        float x, y;
    };

    float triangleArea(Point p1, Point p2, Point p3);
    float isInTriangle(Point p, Point p1, Point p2, Point p3);
}
#endif