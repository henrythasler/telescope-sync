#include <telescope.h>

Telescope::Telescope(void)
{
}

Telescope::Telescope(double ra, double dec)
{
    position.ra = ra;
    position.dec = dec;
}

double Telescope::rad(double degrees)
{
    return (degrees * M_PI / 180);
}

double Telescope::deg(double radians)
{
    return (radians * 180 / M_PI);
}

double Telescope::degToHours(double degrees)
{
    return std::fmod(degrees / 360 * 24, 24.0);
}

void Telescope::fromHorizontalPosition(double azimuth, double altitude, double latitude, double localSiderealTimeDegrees)
{
    double az = rad(azimuth);
    double el = rad(altitude);
    double phi = rad(latitude);

    double sa = sin(az);
    double ca = cos(az);
    double se = sin(el);
    double ce = cos(el);
    double sp = sin(phi);
    double cp = cos(phi);

    double x = -ca * ce * sp + se * cp;
    double y = -sa * ce;
    double z = ca * ce * cp + se * sp;

    double r = sqrt(x * x + y * y);
    double ha = (r != 0.0) ? atan2(y, x) : 0.0;
    position.dec = deg(atan2(z, r));
    position.ra = std::fmod(localSiderealTimeDegrees - deg(ha), 360);
}

Telescope::Horizontal Telescope::toHorizontalPosition(double latitude, double localSiderealTimeDegrees)
{
    Horizontal result;
    double hourAngle = localSiderealTimeDegrees - position.ra;
    double ha = hourAngle >= 0 ? rad(hourAngle) : rad(hourAngle + 360);
    
    double dec = rad(position.dec);
    double phi = rad(latitude);
    
    double sh = sin(ha);
    double ch = cos(ha);
    double sd = sin(dec);
    double cd = cos(dec);
    double sp = sin(phi);
    double cp = cos(phi);

    double x = - ch*cd*sp + sd*cp;
    double y = - sh*cd;
    double z = ch*cd*cp + sd*sp;

    double r = sqrt(x*x + y*y);
    
    double a = a = (r != 0.0) ? atan2(y,x) : 0.0;
    double az = (a < 0.0) ? a+2*M_PI : a;
    double el = atan2(z,r);
    result.az=deg(az);
    result.alt = deg(el);
    return(result);
}
