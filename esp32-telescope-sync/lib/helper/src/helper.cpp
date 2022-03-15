#include <helper.h>

namespace MathHelper
{
    float julianDay(tm timestamp)
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

    // need double here for better precision with the multiplications and all
    double getLocalSiderealTimeDegrees(tm utcTimestamp, double longitude)
    {
        double deltaJulian = julianDay(utcTimestamp) - 2451545L + utcTimestamp.tm_hour / 24.0L + utcTimestamp.tm_min / 60.0L / 24.0L + utcTimestamp.tm_sec / 3600.0L / 24.0L;
        double julianCenturies = deltaJulian / 36525.0L;
        return (f_mod(280.46061837L + 360.98564736629L * deltaJulian + 0.000388L * julianCenturies * julianCenturies + longitude, 360.0L));
    }

    double f_mod(double a, double n)
    {
        return (a - n * floor(a / n));
    }
}

namespace Checksum
{
    bool verifyAmtCheckbits(uint32_t data)
    {
        uint32_t even = data & 0x5555;
        uint32_t odd = data & 0xAAAA;

        uint32_t even_parity = !(((even >> 12) & 0x01) ^ ((even >> 10) & 0x01) ^ ((even >> 8) & 0x01) ^ ((even >> 6) & 0x01) ^ ((even >> 4) & 0x01) ^ ((even >> 2) & 0x01) ^ (even & 0x01));
        uint32_t odd_parity = !(((odd >> 13) & 0x01) ^ ((odd >> 11) & 0x01) ^ ((odd >> 9) & 0x01) ^ ((odd >> 7) & 0x01) ^ ((odd >> 5) & 0x01) ^ ((odd >> 3) & 0x01) ^ ((odd >> 1) & 0x01));

        uint32_t K1 = (data >> 15) & 0x01;
        uint32_t K0 = (data >> 14) & 0x01;
        return ((even_parity == K0) && (odd_parity == K1));
    }
}
