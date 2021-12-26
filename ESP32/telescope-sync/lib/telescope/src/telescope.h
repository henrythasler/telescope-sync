#ifndef TELESCOPE_H
#define TELESCOPE_H

using namespace std;

#include <math.h>

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
        double alt = 0;  // in degrees
        double az = 0; // in degrees
    } Horizontal;

    Equatorial position;
    Equatorial offset;

    Telescope(void);
    Telescope(double ra, double dec);

    double rad(double degrees);
    double deg(double radians);
    double degToHours(double degrees);
    void fromHorizontalPosition(double azimuth, double altitude, double latitude, double localSiderealTimeDegrees);
    Horizontal toHorizontalPosition(double latitude, double localSiderealTimeDegrees);

};
#endif