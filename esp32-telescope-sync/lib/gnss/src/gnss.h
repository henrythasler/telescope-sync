#ifndef GNSS_H
#define GNSS_H

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
    GNSS(float initialLatitude, float initialLongitude);

    bool verifyChecksum(std::string sentence);

    bool fromRMC(std::string sentence);
    bool fromGGA(std::string sentence);
    bool fromGSV(std::string sentence);

    bool fromNMEA(std::string sentence);

    uint32_t fromBuffer(uint8_t *buffer, size_t length);

    tm utcTimestamp = {0};

    bool valid = false;
    uint32_t satUsed = 0;
    uint32_t satView = 0;
    float dilution = 0.0;

    float latitude = 0.0;
    float longitude = 0.0;
    bool north = true;
    bool east = true;
    float speed = 0.0;
    float course = 0.0;
    float altitude = 0.0;    

private:
    const std::string RMC_HEADER = "$GPRMC";
    const std::string GGA_HEADER = "$GPGGA";
    const std::string GSV_HEADER = "$GPGSV";
};
#endif