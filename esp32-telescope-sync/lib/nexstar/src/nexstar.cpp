#include <nexstar.h>

NexStar::NexStar(Telescope *telescope, GNSS *gnss)
{
    this->telescope = telescope;
    this->gnss = gnss;
}

uint32_t NexStar::handleRequest(uint8_t *request, size_t requestLength, uint8_t *response, size_t responseMaxLength)
{
    // Echo
    if (requestLength >= 2 && request[0] == 'K')
    {
        return snprintf((char *)response, responseMaxLength, "%c#", request[1]);
    }
    // Get Version
    else if (requestLength >= 1 && request[0] == 'V')
    {
        return snprintf((char *)response, responseMaxLength, "%c%c#", 4, 10);
    }
    // Get Device Version
    else if (requestLength >= 8 && request[0] == 'P' && request[1] == 0x01 &&
             (request[2] == 0x10 || request[2] == 0x11 || request[2] == 0xb0 || request[2] == 0xb2) &&
             request[3] == 0xfe && request[4] == 0x00 && request[5] == 0x00 && request[6] == 0x00 && request[7] == 0x02) // Get Version
    {
        return snprintf((char *)response, responseMaxLength, "%c%c#", 1, 0);
    }
    // Get Model
    else if (requestLength >= 1 && request[0] == 'm')
    {
        return snprintf((char *)response, responseMaxLength, "%c#", 12);
    }
    // Get Location
    // 57 30 07 3b 00  0b 16 20 00
    // W  48  7 59 N   11 22 32 E
    else if (requestLength >= 1 && request[0] == 'w')
    {
        return snprintf((char *)response, responseMaxLength, "%c%c%c%c%c%c%c%c#",
                        uint8_t(abs(floor(gnss->latitude))),
                        uint8_t((abs(gnss->latitude) - abs(floor(gnss->latitude))) * 60),
                        0,
                        uint8_t(!gnss->north),
                        uint8_t(abs(floor(gnss->longitude))),
                        uint8_t((abs(gnss->longitude) - abs(floor(gnss->longitude))) * 60),
                        0,
                        uint8_t(!gnss->east));
    }
    // Get precise RA/DEC
    else if (requestLength >= 1 && request[0] == 'e')
    {
        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(this->gnss->utcTimestamp, this->gnss->longitude);
        Equatorial position = this->telescope->getCalibratedOrientation(this->gnss->latitude, localSiderealTimeDegrees);
        // Telescope::Equatorial position = this->telescope->horizontalToEquatorial(corrected, this->gnss->latitude, localSiderealTimeDegrees);
        return snprintf((char *)response, responseMaxLength, "%08X,%08X#",
                        uint32_t(position.ra / 360.0L * 4294967296.0L),
                        uint32_t(position.dec / 360.0L * 4294967296.0L));
    }
    // Get Time
    else if (requestLength >= 1 && request[0] == 'h')
    {
        return snprintf((char *)response, responseMaxLength, "%c%c%c%c%c%c%c%c#",
                        gnss->utcTimestamp.tm_hour,
                        gnss->utcTimestamp.tm_min,
                        gnss->utcTimestamp.tm_sec,
                        gnss->utcTimestamp.tm_mon,
                        gnss->utcTimestamp.tm_mday,
                        gnss->utcTimestamp.tm_year - 2000,
                        0,
                        0);
    }
    // Is GOTO in Progress
    else if (requestLength >= 1 && request[0] == 'L')
    {
        return snprintf((char *)response, responseMaxLength, "%c#", 0);
    }
    // Is Alignment Complete?
    else if (requestLength >= 1 && request[0] == 'J')
    {
        return snprintf((char *)response, responseMaxLength, "%c#", telescope->isCalibrated);
    }
    // Get Tracking Mode
    else if (requestLength >= 1 && request[0] == 't')
    {
        return snprintf((char *)response, responseMaxLength, "%c#", 1);
    }
    // Sync precise RA/DEC (e.g. 's2DE3C3B7,0F20C28D')
    else if (requestLength >= 18 && request[0] == 's')
    {
        char hexString[9] = {0};
        Equatorial reference;

        strncpy(hexString, (char *)request + 1, sizeof(hexString) - 1);
        reference.ra = (double)strtoul(hexString, NULL, 16) * 360L / 4294967296.0L;

        strncpy(hexString, (char *)request + 10, sizeof(hexString) - 1);
        reference.dec = (double)strtoul(hexString, NULL, 16) * 360L / 4294967296.0L;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(this->gnss->utcTimestamp, this->gnss->longitude);
        telescope->addReferencePoint(reference, this->gnss->latitude, localSiderealTimeDegrees);
        telescope->isCalibrated = true;
        return snprintf((char *)response, responseMaxLength, "#");
    }
    return 0;
}
