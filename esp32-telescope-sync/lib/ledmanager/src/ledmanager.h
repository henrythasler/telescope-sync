#ifndef LEDMANAGER_H
#define LEDMANAGER_H

using namespace std;

#include <Arduino.h>

class LEDManager
{
public:
    typedef enum
    {
        OFF,
        ON,
        ON_500_OFF_500,
        OFF_3800_ON_10_OFF_190_ON_10_OFF_190_ON_10_OFF_190_ON_10_OFF_190,
        OFF_3800_ON_10_OFF_190_ON_10_OFF_190,
        ON_4990_ON_10,
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

#endif // LEDMANAGER_H