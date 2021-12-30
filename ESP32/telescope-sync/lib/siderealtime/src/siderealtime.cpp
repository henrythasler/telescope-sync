#include <siderealtime.h>

SiderealTime::SiderealTime()
{
}

float SiderealTime::julianDay(tm timestamp)
{
    if (timestamp.tm_mon <= 2)
    {
        timestamp.tm_year -= 1;
        timestamp.tm_mon += 12;
    }

    float A = floor(timestamp.tm_year / 100.0f);
    float B = 2u - A + floor(A / 4.0f);
    return (floor(365.25f * (timestamp.tm_year + 4716)) + floor(30.6001 * (timestamp.tm_mon + 1)) + timestamp.tm_mday + B - 1524.5);
}

float getLocalSiderealTimeDegrees(tm utcTimestamp, float longitude)
{
}