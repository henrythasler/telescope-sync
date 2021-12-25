#include <persistency.h>

Persistency::Persistency(void)
{
}

#ifdef ARDUINO
bool Persistency::readBinaryData(uint8_t *data, size_t length, const char *filename)
{
    File file = SPIFFS.open(filename, FILE_READ);
    if (file && (file.size() >= length))
    {
        size_t bytes = file.read((uint8_t *)&data[0], length);
        file.close();
        if (bytes == length)
        {
            return true;
        }
    }
    else
    {
        Serial.printf("[ ERROR  ] Error opening '%s'\n", filename);
    }
    return false;
}

bool Persistency::writeBinaryData(uint8_t *data, size_t length, const char *filename)
{
    File file = SPIFFS.open(filename, FILE_WRITE);
    if (file)
    {
        size_t bytes = file.write((const uint8_t *)&data[0], length);
        file.close();

        if (bytes == length)
        {
            return true;
        }
    }
    else
    {
        Serial.printf("[ ERROR  ] Error opening '%s'\n", filename);
    }
    return false;
}
#endif