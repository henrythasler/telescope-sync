#include <telescope.h>

Telescope::Telescope(void)
{
}

Telescope::Telescope(double alt, double az)
{
    this->orientation.alt = alt;
    this->orientation.az = az;
}

void Telescope::setOrientation(double alt, double az)
{
    this->orientation.alt = alt;
    this->orientation.az = az;
}

void Telescope::calibrate(Equatorial reference, double latitude, double localSiderealTimeDegrees)
{
    Horizontal hRef = this->equatorialToHorizontal(reference, latitude, localSiderealTimeDegrees);
    this->offset.alt = hRef.alt - this->orientation.alt;
    this->offset.az = hRef.az - this->orientation.az;
}

void Telescope::addReferencePoint(Equatorial *reference, double latitude, double localSiderealTimeDegrees)
{
    memcpy(&this->referencePoints[this->alignmentWritePointer], reference, sizeof *reference);
    this->horizontalToEquatorial(this->orientation, latitude, localSiderealTimeDegrees, &this->actualPoints[this->alignmentWritePointer]);

    this->alignmentWritePointer = (this->alignmentWritePointer + 1) % MAX_ALIGNMENT_POINTS;
    this->alignmentPoints = std::min(this->alignmentPoints + 1, MAX_ALIGNMENT_POINTS);
}

BLA::Matrix<3, 3, BLA::Array<3, 3, double>> Telescope::getTransformationMatrix(uint32_t triangleOffset)
{
    BLA::Matrix<3, 3, BLA::Array<3, 3, double>> transformMatrix = BLA::Identity<3, 3>();

    if (this->alignmentPoints == 1)
    {
        transformMatrix(0, 2) = this->actualPoints[0].dec - this->referencePoints[0].dec;
        transformMatrix(1, 2) = this->actualPoints[0].ra - this->referencePoints[0].ra;
    }
    else if (this->alignmentPoints == 2)
    {
        BLA::Matrix<3, 3, BLA::Array<3, 3, double>> ref = {
            this->referencePoints[0].dec, this->referencePoints[1].dec, this->referencePoints[1].dec + (this->referencePoints[1].ra - this->referencePoints[0].ra),
            this->referencePoints[0].ra, this->referencePoints[1].ra, this->referencePoints[1].ra - (this->referencePoints[1].dec - this->referencePoints[0].dec),
            1, 1, 1};

        BLA::Matrix<3, 3, BLA::Array<3, 3, double>> mount = {
            this->actualPoints[0].dec, this->actualPoints[1].dec, this->actualPoints[1].dec + (this->actualPoints[1].ra - this->actualPoints[0].ra),
            this->actualPoints[0].ra, this->actualPoints[1].ra, this->actualPoints[1].ra - (this->actualPoints[1].dec - this->actualPoints[0].dec),
            1, 1, 1};

        if (BLA::Invert(mount))
            transformMatrix = ref * mount;
    }

    else if (this->alignmentPoints > 2 && (triangleOffset < this->alignmentPoints - 2))
    {
        BLA::Matrix<3, 3, BLA::Array<3, 3, double>> ref = {
            this->referencePoints[triangleOffset + 0].dec, this->referencePoints[triangleOffset + 1].dec, this->referencePoints[triangleOffset + 2].dec,
            this->referencePoints[triangleOffset + 0].ra, this->referencePoints[triangleOffset + 1].ra, this->referencePoints[triangleOffset + 2].ra,
            1, 1, 1};

        BLA::Matrix<3, 3, BLA::Array<3, 3, double>> mount = {
            this->actualPoints[triangleOffset + 0].dec, this->actualPoints[triangleOffset + 1].dec, this->actualPoints[triangleOffset + 2].dec,
            this->actualPoints[triangleOffset + 0].ra, this->actualPoints[triangleOffset + 1].ra, this->actualPoints[triangleOffset + 2].ra,
            1, 1, 1};

        if (BLA::Invert(mount))
            transformMatrix = ref * mount;
    }

    return transformMatrix;
}

Telescope::Horizontal Telescope::getCalibratedOrientation(void)
{
    Telescope::Horizontal result;
    result.alt = this->offset.alt + this->orientation.alt;
    result.az = this->offset.az + this->orientation.az;
    return result;
}

Telescope::Horizontal Telescope::getCalibratedOrientation(BLA::Matrix<3, 3, BLA::Array<3, 3, double>> M)
{
    Telescope::Horizontal result;
    BLA::Matrix<3, 1, BLA::Array<3, 1, double>> in = {this->orientation.az, this->orientation.alt, 1};
    BLA::Matrix<3, 1, BLA::Array<3, 1, double>> out = M * in;
    result.az = out(0);
    result.alt = out(1);
    return result;
}

double Telescope::rad(double degrees)
{
    return (degrees * PI / 180);
}

double Telescope::deg(double radians)
{
    return (radians * 180 / PI);
}

double Telescope::degToHours(double degrees)
{
    return MathHelper::f_mod(degrees / 360 * 24, 24.0);
}

/**
 * Converts the given horizontal coordinates to equatorial coordinates and stores them in the position-property
 * based on: iauAe2hd() from http://www.iausofa.org
 * according to SOFA Software License this function contains modifications regarding input/output types
 * @param latitude in decimal degrees
 * @param localSiderealTimeDegrees 
 * @returns a struct containing the ALT/AZ values
 */
void Telescope::horizontalToEquatorial(double altitude, double azimuth, double latitude, double localSiderealTimeDegrees, Telescope::Equatorial *result)
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
    result->dec = deg(atan2(z, r));
    result->ra = MathHelper::f_mod(localSiderealTimeDegrees - deg(ha), 360);
}

void Telescope::horizontalToEquatorial(Horizontal horizontal, double latitude, double localSiderealTimeDegrees, Telescope::Equatorial *result)
{
    this->horizontalToEquatorial(horizontal.alt, horizontal.az, latitude, localSiderealTimeDegrees, result);
}

Telescope::Equatorial Telescope::horizontalToEquatorial(double altitude, double azimuth, double latitude, double localSiderealTimeDegrees)
{
    Telescope::Equatorial equatorial;
    this->horizontalToEquatorial(altitude, azimuth, latitude, localSiderealTimeDegrees, &equatorial);
    return equatorial;
}

Telescope::Equatorial Telescope::horizontalToEquatorial(Horizontal horizontal, double latitude, double localSiderealTimeDegrees)
{
    Telescope::Equatorial equatorial;
    this->horizontalToEquatorial(horizontal.alt, horizontal.az, latitude, localSiderealTimeDegrees, &equatorial);
    return equatorial;
}

/**
 * Converts the current position to horizontal coordinates
 * based on: iauHd2ae() from http://www.iausofa.org 
 * according to SOFA Software License this function contains modifications regarding input/output types.
 * @param ra in decimal degrees
 * @param dec in decimal degrees
 * @param latitude in decimal degrees
 * @param localSiderealTimeDegrees 
 * @returns a struct containing the ALT/AZ values
 */
void Telescope::equatorialToHorizontal(double ra, double dec, double latitude, double localSiderealTimeDegrees, Telescope::Horizontal *result)
{
    double hourAngle = localSiderealTimeDegrees - ra;
    double ha = hourAngle >= 0 ? rad(hourAngle) : rad(hourAngle + 360);

    dec = rad(dec);
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
    double az = (a < 0.0) ? a + 2 * PI : a;
    double el = atan2(z, r);
    result->az = deg(az);
    result->alt = deg(el);
}

void Telescope::equatorialToHorizontal(Equatorial equatorial, double latitude, double localSiderealTimeDegrees, Telescope::Horizontal *result)
{
    this->equatorialToHorizontal(equatorial.ra, equatorial.dec, latitude, localSiderealTimeDegrees, result);
}

Telescope::Horizontal Telescope::equatorialToHorizontal(Equatorial equatorial, double latitude, double localSiderealTimeDegrees)
{
    Telescope::Horizontal result;
    this->equatorialToHorizontal(equatorial.ra, equatorial.dec, latitude, localSiderealTimeDegrees, &result);
    return result;
}

Telescope::Horizontal Telescope::equatorialToHorizontal(double ra, double dec, double latitude, double localSiderealTimeDegrees)
{
    Telescope::Horizontal result;
    this->equatorialToHorizontal(ra, dec, latitude, localSiderealTimeDegrees, &result);
    return result;
}

/**
 * Serialized the current position (ra/dec) and timestamp into a given memory location (buffer).
 * Serialisation for MessageCurrentPosition of Stellarium Telescope Protocol version 1.0
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param buffer pointer to a memory location that will hold the serialized data
 * @param bufferSize size of the buffer to check for OOB-access
 * @returns the number of bytes that were serialized. 24 for OK, anything else for ERROR.
 */
uint32_t Telescope::packPosition(double ra, double dec, uint64_t timestamp, uint8_t *buffer, size_t bufferSize)
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
    encoded = static_cast<uint32_t>(floor(0.5 + ra * static_cast<double>(0x80000000) / 180.));
    *writePtr++ = encoded & 0xFF;
    *writePtr++ = (encoded >> 8) & 0xFF;
    *writePtr++ = (encoded >> 16) & 0xFF;
    *writePtr++ = (encoded >> 24) & 0xFF;

    encoded = static_cast<uint32_t>(floor(0.5 + dec * static_cast<double>(0x80000000) / 180.));
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

uint32_t Telescope::packPosition(Equatorial equatorial, uint64_t timestamp, uint8_t *buffer, size_t bufferSize)
{
    return packPosition(equatorial.ra, equatorial.dec, timestamp, buffer, bufferSize);
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

    if (timestamp)
    {
        *timestamp = (static_cast<uint64_t>(data[4]) +
                      (static_cast<uint64_t>(data[5]) << 8) +
                      (static_cast<uint64_t>(data[6]) << 16) +
                      (static_cast<uint64_t>(data[7]) << 24) +
                      (static_cast<uint64_t>(data[8]) << 32) +
                      (static_cast<uint64_t>(data[9]) << 40) +
                      (static_cast<uint64_t>(data[10]) << 48) +
                      (static_cast<uint64_t>(data[11]) << 56)) /
                     1000;
    }
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
 * Another wrapper for unpackPosition() where the values are stored in a given Equatorial struct 'equatorial' and 'timestamp'
 * @param data pointer to the serialized input data
 * @param dataLength size of the input data to check for OOB-access
 * @returns true for OK, false for ERROR.
 */
bool Telescope::unpackPosition(Equatorial *equatorial, uint64_t *timestamp, uint8_t *data, size_t dataLength)
{
    return unpackPosition(&equatorial->ra, &equatorial->dec, timestamp, data, dataLength);
}
