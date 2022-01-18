#ifndef LEDMANAGER_H
#define LEDMANAGER_H

using namespace std;

#ifdef ARDUINO
#include <Arduino.h>

class LEDManager
{
public:
    typedef enum
    {
        OFF,
        ON,
        BLINK_1HZ,
        FLASH_4X_EVERY_5S,
        FLASH_2X_EVERY_5S,
        FLASH_1X_EVERY_5S,
        READY,
    } LEDMode;

    typedef struct 
    {
        bool startMode;
        uint8_t timing[16];
    } LEDTiming;


    LEDManager(uint8_t pin=LED_BUILTIN, LEDMode mode=LEDMode::OFF);
    void setMode(LEDMode mode);
    void update(uint32_t counter=0);

private:
    uint8_t pin = LED_BUILTIN;
    LEDMode mode = LEDMode::OFF;
    uint32_t step = 0;

    typedef struct
    {
        uint32_t on[6] = {0};
        uint32_t off[6] = {0};
    } Timing;

    Timing timing[32]; // support 32 states
};

#endif
#endif // LEDMANAGER_H