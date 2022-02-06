#include <ledmanager.h>

#ifdef ARDUINO
LEDManager::LEDManager(uint8_t pin, LEDMode mode)
{
    this->pin = pin;
    this->mode = mode;
}

void LEDManager::setMode(LEDMode mode)
{
    this->mode = mode;
};

void LEDManager::update(uint32_t micros)
{
    uint32_t millis = micros / 1000;
    if (this->mode == LEDMode::OFF)
    {
        digitalWrite(this->pin, LOW);
    }

    else if (this->mode == LEDMode::ON)
    {
        digitalWrite(this->pin, HIGH);
    }
    else if (this->mode == LEDMode::BLINK_10HZ)
    {
        if ((millis % 100) < 50)
        {
            digitalWrite(this->pin, HIGH);
        }
        else
        {
            digitalWrite(this->pin, LOW);
        }
    }
    else if (this->mode == LEDMode::BLINK_1HZ)
    {
        if ((millis % 1000) < 500)
        {
            digitalWrite(this->pin, HIGH);
        }
        else
        {
            digitalWrite(this->pin, LOW);
        }
    }
    else if (this->mode == LEDMode::FLASH_1X_EVERY_5S)
    {
        uint32_t seq = millis % 5000;
        if (seq < 4990)
        {
            digitalWrite(this->pin, LOW);
        }
        else
        {
            digitalWrite(this->pin, HIGH);
        }
    }
    else if (this->mode == LEDMode::FLASH_4X_EVERY_5S)
    {
        uint32_t seq = millis % 5000;
        if (seq < 3750)
        {
            digitalWrite(this->pin, LOW);
        }
        else if (seq < 3800)
        {
            digitalWrite(this->pin, HIGH);
        }
        else if (seq < 4150)
        {
            digitalWrite(this->pin, LOW);
        }
        else if (seq < 4200)
        {
            digitalWrite(this->pin, HIGH);
        }
        else if (seq < 4550)
        {
            digitalWrite(this->pin, LOW);
        }
        else if (seq < 4600)
        {
            digitalWrite(this->pin, HIGH);
        }
        else if (seq < 4950)
        {
            digitalWrite(this->pin, LOW);
        }
        else
        {
            digitalWrite(this->pin, HIGH);
        }
    }

    else if (this->mode == LEDMode::FLASH_2X_EVERY_5S)
    {
        uint32_t seq = millis % 5000;
        if (seq < 4550)
        {
            digitalWrite(this->pin, LOW);
        }
        else if (seq < 4600)
        {
            digitalWrite(this->pin, HIGH);
        }
        else if (seq < 4950)
        {
            digitalWrite(this->pin, LOW);
        }
        else
        {
            digitalWrite(this->pin, HIGH);
        }
    }
};
#endif