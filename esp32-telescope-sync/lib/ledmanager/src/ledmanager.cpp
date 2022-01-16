#include <ledmanager.h>

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

    else if(this->mode == LEDMode::ON_500_OFF_500) {
        if((this->step % 100) < 50) {
            digitalWrite(this->pin, HIGH);
        }
        else {
            digitalWrite(this->pin, LOW); 
        }
    }
    else if(this->mode == LEDMode::ON_4990_ON_10) {
        uint32_t seq = this->step % 500;
        if(seq < 4990) {
            digitalWrite(this->pin, LOW);
        }
        else {
            digitalWrite(this->pin, HIGH); 
        }
    }
    else if(this->mode == LEDMode::OFF_3800_ON_10_OFF_190_ON_10_OFF_190_ON_10_OFF_190_ON_10_OFF_190) {
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

    else if(this->mode == LEDMode::OFF_3800_ON_10_OFF_190_ON_10_OFF_190) {
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

