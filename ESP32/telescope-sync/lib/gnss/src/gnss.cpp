#include <gnss.h>

GNSS::GNSS()
{
}

bool GNSS::fromRMC(string sentence)
{
    int32_t pos = 0;
    char *ptr;
    float rawValue = 0.0;

    if (!this->verifyChecksum(sentence))
        return false;

    pos = sentence.find(RMC_HEADER);
    if (pos == string::npos)
        return false;

    float rawTimestamp = strtof(sentence.c_str() + pos + RMC_HEADER.length() + 1, &ptr);
    this->utcTimestamp.tm_hour = floor(rawTimestamp / 10000.0f);
    this->utcTimestamp.tm_min = floor(rawTimestamp / 100.0f) - utcTimestamp.tm_hour * 100.0f;
    this->utcTimestamp.tm_sec = floor(rawTimestamp) - utcTimestamp.tm_hour * 10000.0f - utcTimestamp.tm_min * 100.0f;

    this->valid = (*(++ptr) == 'A');

    ptr += (*ptr == ',') ? 1 : 2;
    rawValue = strtof(ptr, &ptr);
    this->latitude = floor(rawValue / 100.0f) + (rawValue - floor(rawValue / 100.0f) * 100.0f) / 60.0f; // convert to decimal degrees

    this->north = (*(++ptr) == 'N');
    this->latitude *= this->north ? 1 : -1; // set sign accoringly (north=positive; south=negative)

    ptr += (*ptr == ',') ? 1 : 2;
    rawValue = strtof(ptr, &ptr);
    this->longitude = floor(rawValue / 100.0f) + (rawValue - floor(rawValue / 100.0f) * 100.0f) / 60.0f;

    this->east = (*(++ptr) == 'E');
    this->longitude *= this->east ? 1 : -1; // set sign accoringly (east=positive; west=negative)

    ptr += (*ptr == ',') ? 1 : 2;
    rawValue = strtof(ptr, &ptr);
    this->speed = rawValue / 1.9438444924f; // convert to m/s

    ptr++;
    this->course = strtof(ptr, &ptr);

    ptr++;
    uint32_t rawDate = strtol(ptr, &ptr, 10);
    this->utcTimestamp.tm_mday = rawDate / 10000L;
    this->utcTimestamp.tm_mon = rawDate / 100L - this->utcTimestamp.tm_mday * 100L;
    this->utcTimestamp.tm_year = rawDate - this->utcTimestamp.tm_mday * 10000L - this->utcTimestamp.tm_mon * 100L + 2000;

    // skip Magnetic Variation and Mode for now

    return true;
}

/**
 * Example: "GPGGA,083055.00,4815.69961,N,01059.02625,E,1,04,7.80,485.8,M,46.8,M,,*50"
 **/

bool GNSS::fromGGA(string sentence)
{
    int32_t pos = 0;
    char *ptr;
    float rawValue = 0.0;
    uint32_t rawLong = 0;

    if (!this->verifyChecksum(sentence))
        return false;

    pos = sentence.find(GGA_HEADER);
    if (pos == string::npos)
        return false;

    float rawTimestamp = strtof(sentence.c_str() + pos + RMC_HEADER.length() + 1, &ptr);
    this->utcTimestamp.tm_hour = floor(rawTimestamp / 10000.0f);
    this->utcTimestamp.tm_min = floor(rawTimestamp / 100.0f) - utcTimestamp.tm_hour * 100.0f;
    this->utcTimestamp.tm_sec = floor(rawTimestamp) - utcTimestamp.tm_hour * 10000.0f - utcTimestamp.tm_min * 100.0f;

    ptr++;
    rawValue = strtof(ptr, &ptr);
    this->latitude = floor(rawValue / 100.0f) + (rawValue - floor(rawValue / 100.0f) * 100.0f) / 60.0f; // convert to decimal degrees

    ptr++;
    this->north = (*ptr == 'N');
    this->latitude *= this->north ? 1 : -1; // set sign accoringly (north=positive; south=negative)

    ptr += (*ptr == ',') ? 1 : 2;
    rawValue = strtof(ptr, &ptr);
    this->longitude = floor(rawValue / 100.0f) + (rawValue - floor(rawValue / 100.0f) * 100.0f) / 60.0f;

    ptr++;
    this->east = (*ptr == 'E');
    this->longitude *= this->east ? 1 : -1; // set sign accoringly (east=positive; west=negative)

    ptr += (*ptr == ',') ? 1 : 2;
    rawLong = strtol(ptr, &ptr, 10);
    this->valid = (rawLong > 0);

    ptr++;
    this->satUsed = strtol(ptr, &ptr, 10);

    ptr++;
    this->dilution = strtof(ptr, &ptr);

    ptr++;
    this->altitude = strtof(ptr, &ptr);

    return true;
}

bool GNSS::fromGSV(string sentence)
{
    int32_t pos = 0;
    char *ptr;

    if (!this->verifyChecksum(sentence))
        return false;

    pos = sentence.find(GSV_HEADER);
    if (pos == string::npos)
        return false;

    // skip "Number of Messages"
    strtol(sentence.c_str() + pos + GSV_HEADER.length() + 1, &ptr, 10);
    ptr++;

    // skip "Sequence Number"
    strtol(ptr, &ptr, 10);
    ptr++;

    this->satView = strtol(ptr, &ptr, 10);

    return true;
}

bool GNSS::verifyChecksum(string sentence)
{
    // verify checksum first
    uint32_t startPos = sentence.find('$');
    uint32_t endPos = sentence.find('*');
    if ((startPos == string::npos) || (endPos == string::npos))
        return false;

    uint32_t expectedChecksum = strtol(sentence.c_str() + endPos + 1, NULL, 16);
    uint32_t actualChecksum = 0;
    for (uint32_t i = startPos + 1; i < endPos; i++)
        actualChecksum ^= sentence.c_str()[i]; // XOR all characters BETWEEN '$' and '*'

    if (expectedChecksum != actualChecksum)
    {
        // printf("%X != %X", expectedChecksum, actualChecksum);
        return false;
    }
    return true;
}

bool GNSS::fromNMEA(string sentence)
{
    if (sentence.find(RMC_HEADER) != string::npos)
        return this->fromRMC(sentence);

    else if (sentence.find(GGA_HEADER) != string::npos)
        return this->fromGGA(sentence);

    else if (sentence.find(GSV_HEADER) != string::npos)
        return this->fromGSV(sentence);

    return false;
}

uint32_t GNSS::fromBuffer(uint8_t *buffer, size_t length)
{
    string rawInput = string(buffer, buffer + length);
    int32_t startPos = 0;
    int32_t endPos = 0;
    int32_t sentences = 0;
    do
    {
        startPos = rawInput.find('$', endPos);
        endPos = rawInput.find('\n', endPos + 1);

        if ((startPos != string::npos) && (endPos != string::npos))
        {
            // printf("'%s' %u..%u\n", rawInput.substr(startPos, endPos - startPos - 1).c_str(), startPos, endPos);
            if (this->fromNMEA(rawInput.substr(startPos, endPos - startPos + 1)))
                sentences++;
        }

    } while ((startPos != string::npos) && (endPos != string::npos));

    return sentences;
}
