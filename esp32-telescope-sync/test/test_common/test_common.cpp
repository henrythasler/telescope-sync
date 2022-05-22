#include <unity.h>
#include <telescope.h>
#include <gnss.h>
#include <nexstar.h>
#include <helper.h>

#include "test_mathhelper.hpp"
#include "test_gnss.hpp"
#include "test_nexstar.hpp"
#include "test_telescope.hpp"
#include "test_alignment.hpp"
#include "test_types.hpp"


void process(void)
{
    Test_Types::process();
    Test_Mathhelper::process();
    Test_Alignment::process();
    Test_GNSS::process();
    Test_Telescope::process();
    Test_Nexstar::process();
}

#ifdef ARDUINO
#include <Arduino.h>

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.printf("Heap: %u KiB free\n", ESP.getFreeHeap() / 1024);
    process();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}

#else

int main(int argc, char **argv)
{
    auto start = std::chrono::high_resolution_clock::now();
    process();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Elapsed time: %lums\n", duration.count() / 1000);
    return 0;
}
#endif