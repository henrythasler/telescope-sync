#ifndef TELESCOPE_H
#define TELESCOPE_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <algorithm>
#include <helper.h>
#include <linalg.h>
#include <mytypes.h>
#include <alignment.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#define PI M_PI
#endif

#define MESSAGE_CURRENT_POSITION_LENGTH (24)
#define MESSAGE_CURRENT_POSITION_TYPE (0)
#define MAX_ALIGNMENT_POINTS (64)

class Telescope
{
private:
public:
    Alignment alignment;

    Horizontal orientation;

    Telescope(void);
    Telescope(double az, double alt);

    void setOrientation(double az, double alt);
    void setOrientation(Horizontal orientation);

    bool isCalibrated = false;
    void addReferencePoint(Equatorial reference, double latitude, double localSiderealTimeDegrees);
    Equatorial getCalibratedOrientation(double latitude, double localSiderealTimeDegrees, TransformationType &transformationType);
    Equatorial getCalibratedOrientation(double latitude, double localSiderealTimeDegrees);

    double rad(double degrees);
    double deg(double radians);
    double degToHours(double degrees);

    void horizontalToEquatorial(double azimuth, double altitude, double latitude, double localSiderealTimeDegrees, Equatorial *result);
    void horizontalToEquatorial(Horizontal horizontal, double latitude, double localSiderealTimeDegrees, Equatorial *result);
    Equatorial horizontalToEquatorial(Horizontal horizontal, double latitude, double localSiderealTimeDegrees);
    Equatorial horizontalToEquatorial(double azimuth, double altitude, double latitude, double localSiderealTimeDegrees);

    void equatorialToHorizontal(double ra, double dec, double latitude, double localSiderealTimeDegrees, Horizontal *result);
    void equatorialToHorizontal(Equatorial equatorial, double latitude, double localSiderealTimeDegrees, Horizontal *result);
    Horizontal equatorialToHorizontal(Equatorial equatorial, double latitude, double localSiderealTimeDegrees);
    Horizontal equatorialToHorizontal(double ra, double dec, double latitude, double localSiderealTimeDegrees);

    uint32_t packPosition(double ra, double dec, uint64_t timestamp, uint8_t *buffer, size_t bufferSize);
    uint32_t packPosition(Equatorial equatorial, uint64_t timestamp, uint8_t *buffer, size_t bufferSize);

    bool unpackPosition(double *ra, double *dec, uint64_t *timestamp, uint8_t *data, size_t dataLength = 20);
    bool unpackPosition(Equatorial *equatorial, uint64_t *timestamp, uint8_t *data, size_t dataLength);
};

#endif
