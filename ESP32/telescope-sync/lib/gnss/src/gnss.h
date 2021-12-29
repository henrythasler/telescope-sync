#ifndef GNSS_H
#define GNSS_H

using namespace std;

#include <stdint.h>
#include <ctime>
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <string>
#endif

class GNSS
{
public:
    GNSS();

    bool fromGPRMC(string gprmc, size_t length);

    tm utcTimestamp;
    bool valid = false;
    float latitude = 0.0;
    float longitude = 0.0;
    bool north = true;
    bool east = true;
    float speed = 0.0;
    float course = 0.0;

private:
    const std::string GPRMC_HEADER = "$GPRMC,";
};
#endif