#ifndef SIDEREALTIME_H
#define SIDEREALTIME_H

using namespace std;

#include <stdint.h>
#include <ctime>
#include <math.h>

class SiderealTime
{
public:
    SiderealTime();
    float getLocalSiderealTimeDegrees(tm utcTimestamp, float longitude);
    float julianDay(tm timestamp);

private:
};
#endif