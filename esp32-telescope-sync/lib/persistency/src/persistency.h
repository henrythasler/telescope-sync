#ifndef PERSISTENCY_H
#define PERSISTENCY_H

#ifdef ARDUINO
#include <SPIFFS.h>
#endif

#include <stdint.h>
#include <stddef.h>

class Persistency
{
private:
public:
    Persistency(void);
    bool readBinaryData(uint8_t *data, size_t length, const char *filename);
    bool writeBinaryData(uint8_t *data, size_t length, const char *filename);
};
#endif