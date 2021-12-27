#include <telescope.h>

Telescope::Telescope(void)
{
}

Telescope::Telescope(double ra, double dec)
{
    position.ra = ra;
    position.dec = dec;
}

void Telescope::setPosition(double ra, double dec)
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
    return fmodf(degrees / 360 * 24, 24.0);
}

/**
 * Converts the given horizontal coordinates to equatorial coordinates and stores them in the position-property
 * based on: iauAe2hd() from http://www.iausofa.org
 * according to SOFA Software License this function contains modifications regarding input/output types
 * @param latitude in decimal degrees
 * @param localSiderealTimeDegrees 
 * @returns a struct containing the ALT/AZ values
 */
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
    position.ra = fmodf(localSiderealTimeDegrees - deg(ha), 360);
}

/**
 * Converts the current position to horizontal coordinates
 * based on: iauHd2ae() from http://www.iausofa.org 
 * according to SOFA Software License this function contains modifications regarding input/output types.
 * @param latitude in decimal degrees
 * @param localSiderealTimeDegrees 
 * @returns a struct containing the ALT/AZ values
 */
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

    double x = -ch * cd * sp + sd * cp;
    double y = -sh * cd;
    double z = ch * cd * cp + sd * sp;

    double r = sqrt(x * x + y * y);

    double a = (r != 0.0) ? atan2(y, x) : 0.0;
    double az = (a < 0.0) ? a + 2 * M_PI : a;
    double el = atan2(z, r);
    result.az = deg(az);
    result.alt = deg(el);
    return (result);
}

/**
 * Serialized the current position (ra/dec) and timestamp into a given memory location (buffer).
 * Serialisation for MessageCurrentPosition of Stellarium Telescope Protocol version 1.0
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param buffer pointer to a memory location that will hold the serialized data
 * @param bufferSize size of the buffer to check for OOB-access
 * @returns the number of bytes that were serialized. 24 for OK, anything else for ERROR.
 */
uint32_t Telescope::packPosition(uint8_t *buffer, size_t bufferSize)
{
    uint32_t encoded = 0;

    uint8_t *writePtr = buffer;

    if (bufferSize < 24)
    {
        return 0;
    }

    // LENGTH
    *writePtr++ = MESSAGE_CURRENT_POSITION_LENGTH & 0xFF;
    *writePtr++ = (MESSAGE_CURRENT_POSITION_LENGTH >> 8) & 0xFF;

    // TYPE
    *writePtr++ = MESSAGE_CURRENT_POSITION_TYPE & 0xFF;
    *writePtr++ = (MESSAGE_CURRENT_POSITION_TYPE >> 8) & 0xFF;

    // TIME
    *writePtr++ = timestamp & 0xFF;
    ;
    *writePtr++ = (timestamp >> 8) & 0xFF;
    *writePtr++ = (timestamp >> 16) & 0xFF;
    *writePtr++ = (timestamp >> 24) & 0xFF;
    *writePtr++ = (timestamp >> 32) & 0xFF;
    *writePtr++ = (timestamp >> 40) & 0xFF;
    *writePtr++ = (timestamp >> 48) & 0xFF;
    *writePtr++ = (timestamp >> 56) & 0xFF;

    // this includes conversion from degrees to hours
    encoded = static_cast<uint32_t>(floor(0.5 + position.ra * static_cast<double>(0x80000000) / 180.));
    *writePtr++ = encoded & 0xFF;
    *writePtr++ = (encoded >> 8) & 0xFF;
    *writePtr++ = (encoded >> 16) & 0xFF;
    *writePtr++ = (encoded >> 24) & 0xFF;

    encoded = static_cast<uint32_t>(floor(0.5 + position.dec * static_cast<double>(0x80000000) / 180.));
    *writePtr++ = encoded & 0xFF;
    *writePtr++ = (encoded >> 8) & 0xFF;
    *writePtr++ = (encoded >> 16) & 0xFF;
    *writePtr++ = (encoded >> 24) & 0xFF;

    // STATUS (0=OK)
    *writePtr++ = 0;
    *writePtr++ = 0;
    *writePtr++ = 0;
    *writePtr++ = 0;

    return (writePtr - buffer);
}

/**
 * Deserialized MessageGoto according to MessageCurrentPosition of Stellarium Telescope Protocol version 1.0
 * all values are deserialized as degrees
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * and https://github.com/Stellarium/stellarium/blob/29df37b6c590ed32266f8269fde94bcb95eb5aa1/plugins/TelescopeControl/src/TelescopeClient.cpp#L276
 * @param ra pointer to a memory location where the RA-value will be deserialized into
 * @param dec pointer to a memory location where the DEC-value will be deserialized into
 * @param timestamp pointer to a memory location where the TIME-value will be deserialized into
 * @param data pointer to the serialized input data
 * @param dataLength size of the input data to check for OOB-access
 * @returns true for OK, false for ERROR.
 */
bool Telescope::unpackPosition(double *ra, double *dec, uint64_t *timestamp, uint8_t *data, size_t dataLength)
{
    uint32_t length = static_cast<uint32_t>(data[0]) + static_cast<uint32_t>(data[1] << 8);
    if ((length < 20) || (dataLength < length))
        return false;

    uint32_t type = static_cast<uint32_t>(data[2]) + static_cast<uint32_t>(data[3] << 8);
    if (type != 0)
        return false;

    *timestamp = (static_cast<uint64_t>(data[4]) +
                  (static_cast<uint64_t>(data[5]) << 8) +
                  (static_cast<uint64_t>(data[6]) << 16) +
                  (static_cast<uint64_t>(data[7]) << 24) +
                  (static_cast<uint64_t>(data[8]) << 32) +
                  (static_cast<uint64_t>(data[9]) << 40) +
                  (static_cast<uint64_t>(data[10]) << 48) +
                  (static_cast<uint64_t>(data[11]) << 56)) /
                 1000;

    *ra = static_cast<double>(static_cast<uint32_t>(data[12]) +
                              (static_cast<uint32_t>(data[13]) << 8) +
                              (static_cast<uint32_t>(data[14]) << 16) +
                              (static_cast<uint32_t>(data[15]) << 24)) /
          static_cast<double>(0x80000000) * 180.;

    *dec = static_cast<double>(static_cast<uint32_t>(data[16]) +
                               (static_cast<uint32_t>(data[17]) << 8) +
                               (static_cast<uint32_t>(data[18]) << 16) +
                               (static_cast<uint32_t>(data[19]) << 24)) /
           static_cast<double>(0x80000000) * 180.;
    return true;
}

/**
 * Wrapper for unpackPosition() where the values are stored directly in the class properties 'position' and 'timestamp'
 * @param data pointer to the serialized input data
 * @param dataLength size of the input data to check for OOB-access
 * @returns true for OK, false for ERROR.
 */
bool Telescope::unpackPosition(uint8_t *data, size_t dataLength)
{
    return unpackPosition(&position.ra, &position.dec, &timestamp, data, dataLength);
}
