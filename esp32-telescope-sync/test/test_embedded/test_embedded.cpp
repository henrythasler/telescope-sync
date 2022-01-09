#include <Arduino.h>
#include <unity.h>

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_led_builtin_pin_number(void)
{
    TEST_ASSERT_EQUAL(LED_BUILTIN, 13);
}

void test_led_state_high(void)
{
    digitalWrite(LED_BUILTIN, HIGH);
    TEST_ASSERT_EQUAL(digitalRead(LED_BUILTIN), HIGH);
}

void test_led_state_low(void)
{
    digitalWrite(LED_BUILTIN, LOW);
    TEST_ASSERT_EQUAL(digitalRead(LED_BUILTIN), LOW);
}


void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_led_builtin_pin_number);
    
    // prepare for I/O test
    pinMode(LED_BUILTIN, OUTPUT);

    RUN_TEST(test_led_state_high);
    RUN_TEST(test_led_state_low);    

    UNITY_END();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);    
}