#include <gnss.h>

GNSS::GNSS()
{
}

bool GNSS::fromGPRMC(string input, size_t length)
{
    int32_t pos = 0;
    char *ptr;
    float rawValue = 0.0;

    pos = input.find(GPRMC_HEADER);
    if (pos == string::npos)
        return false;

    float rawTimestamp = strtof(input.c_str() + GPRMC_HEADER.length(), &ptr);
    this->utcTimestamp.tm_hour = floor(rawTimestamp / 10000);
    this->utcTimestamp.tm_min = floor(rawTimestamp / 100) - utcTimestamp.tm_hour * 100;
    this->utcTimestamp.tm_sec = floor(rawTimestamp) - utcTimestamp.tm_hour * 10000 - utcTimestamp.tm_min * 100;

    this->valid = (*(++ptr) == 'A');

    ptr += 2;
    rawValue = strtof(ptr, &ptr);
    this->latitude = floor(rawValue/100.0f) + (rawValue - floor(rawValue/100.0f) * 100.0f) / 60.0f;

    this->north = (*(++ptr) == 'N');

    ptr += 2;
    rawValue = strtof(ptr, &ptr);
    this->longitude = floor(rawValue/100.0f) + (rawValue - floor(rawValue/100.0f) * 100.0f) / 60.0f;

    this->east = (*(++ptr) == 'E');

    ptr += 2;
    rawValue = strtof(ptr, &ptr);
    this->speed = rawValue / 1.9438444924;  // convert to m/s

    ptr ++;
    this->course = strtof(ptr, &ptr);



#ifdef ARDUINO
#else
#endif

    return true;
}