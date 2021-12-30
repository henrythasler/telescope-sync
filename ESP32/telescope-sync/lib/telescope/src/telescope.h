#ifndef TELESCOPE_H
#define TELESCOPE_H

using namespace std;

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <helper.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#define PI M_PI
#endif

#define MESSAGE_CURRENT_POSITION_LENGTH (24)
#define MESSAGE_CURRENT_POSITION_TYPE (0)

class Telescope
{
private:
public:
    typedef struct
    {
        double ra = 0;  // in degrees
        double dec = 0; // in degrees
    } Equatorial;

    typedef struct
    {
        double alt = 0; // in degrees
        double az = 0;  // in degrees
    } Horizontal;

    Equatorial position;
    Equatorial offset;

    uint64_t timestamp;

    Telescope(void);
    Telescope(double ra, double dec);

    void setPosition(double ra, double dec);
    void calibrate(Equatorial *reference);
    void getCalibratedPosition(Equatorial *calibratedPosition);

    double rad(double degrees);
    double deg(double radians);
    double degToHours(double degrees);
    void fromHorizontalPosition(double azimuth, double altitude, double latitude, double localSiderealTimeDegrees);
    Horizontal toHorizontalPosition(double latitude, double localSiderealTimeDegrees);

    uint32_t packPosition(double *ra, double *dec, uint64_t *timestamp, uint8_t *buffer, size_t bufferSize);
    uint32_t packPosition(Equatorial *equatorial, uint64_t *timestamp, uint8_t *buffer, size_t bufferSize);
    uint32_t packPosition(uint8_t *buffer, size_t bufferSize = 24);

    bool unpackPosition(double *ra, double *dec, uint64_t *timestamp, uint8_t *data, size_t dataLength = 20);
    bool unpackPosition(Equatorial *equatorial, uint64_t *timestamp, uint8_t *data, size_t dataLength);
    bool unpackPosition(uint8_t *data, size_t dataLength = 20);
};
#endif