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

void LEDManager::update(uint32_t counter)
{
    this->step++;

    if(this->mode == LEDMode::OFF) {
            digitalWrite(this->pin, LOW);
    }

    else if(this->mode == LEDMode::ON) {
            digitalWrite(this->pin, HIGH);
    }

    else if(this->mode == LEDMode::BLINK_1HZ) {
        if((this->step % 100) < 50) {
            digitalWrite(this->pin, HIGH);
        }
        else {
            digitalWrite(this->pin, LOW); 
        }
    }
    else if(this->mode == LEDMode::FLASH_1X_EVERY_5S) {
        uint32_t seq = this->step % 500;
        if(seq < 499) {
            digitalWrite(this->pin, LOW);
        }
        else {
            digitalWrite(this->pin, HIGH); 
        }
    }
    else if(this->mode == LEDMode::FLASH_4X_EVERY_5S) {
        uint32_t seq = this->step % 500;
        if(seq < 375) {
            digitalWrite(this->pin, LOW);
        }
        else if(seq < 380) {
            digitalWrite(this->pin, HIGH); 
        }
        else if(seq < 415) {
            digitalWrite(this->pin, LOW);
        }
        else if(seq < 420) {
            digitalWrite(this->pin, HIGH);
        }
        else if(seq < 455) {
            digitalWrite(this->pin, LOW);
        }
        else if(seq < 460) {
            digitalWrite(this->pin, HIGH);
        }
        else if(seq < 495) {
            digitalWrite(this->pin, LOW); 
        }
        else {
            digitalWrite(this->pin, HIGH); 
        }
    }

    else if(this->mode == LEDMode::FLASH_2X_EVERY_5S) {
        uint32_t seq = this->step % 500;
        if(seq < 455) {
            digitalWrite(this->pin, LOW);
        }
        else if(seq < 460) {
            digitalWrite(this->pin, HIGH); 
        }
        else if(seq < 495) {
            digitalWrite(this->pin, LOW); 
        }
        else {
            digitalWrite(this->pin, HIGH); 
        }
    }

};
#endif