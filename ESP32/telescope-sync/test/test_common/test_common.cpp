#include <unity.h>
#include <telescope.h>

#ifndef ARDUINO
#include <stdio.h>
#include <chrono>
#endif

void test_function_rad(void)
{
    Telescope telescope;
    TEST_ASSERT_EQUAL_FLOAT(M_PI_2, telescope.rad(90));
    TEST_ASSERT_EQUAL_FLOAT(M_PI, telescope.rad(180));
    TEST_ASSERT_EQUAL_FLOAT(2 * M_PI, telescope.rad(360));

    TEST_ASSERT_EQUAL_FLOAT(-M_PI_2, telescope.rad(-90));
    TEST_ASSERT_EQUAL_FLOAT(-M_PI, telescope.rad(-180));
    TEST_ASSERT_EQUAL_FLOAT(-2 * M_PI, telescope.rad(-360));
}

void test_function_deg(void)
{
    Telescope telescope;
    TEST_ASSERT_EQUAL_FLOAT(90, telescope.deg(M_PI_2));
    TEST_ASSERT_EQUAL_FLOAT(180, telescope.deg(M_PI));
    TEST_ASSERT_EQUAL_FLOAT(360, telescope.deg(2 * M_PI));

    TEST_ASSERT_EQUAL_FLOAT(-90, telescope.deg(-M_PI_2));
    TEST_ASSERT_EQUAL_FLOAT(-180, telescope.deg(-M_PI));
    TEST_ASSERT_EQUAL_FLOAT(-360, telescope.deg(-2 * M_PI));
}

void test_function_degToHours(void)
{
    Telescope telescope;
    TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(0));
    TEST_ASSERT_EQUAL_FLOAT(6, telescope.degToHours(90));
    TEST_ASSERT_EQUAL_FLOAT(12, telescope.degToHours(180));
    TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(360));
    TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(-360));
}

void test_function_fromHorizontalPosition(void)
{
    Telescope telescope;
    telescope.fromHorizontalPosition(180, 60.34, 38.59, 297.93);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 297.92, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 8.93, telescope.position.dec);

    telescope.fromHorizontalPosition(269.14634, 49.169122, 52.5, 304.80762);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, telescope.position.dec);
}

void test_function_toHorizontalPosition(void)
{
    Telescope telescope(250.425, 36.467);
    Telescope::Horizontal res = telescope.toHorizontalPosition(52.5, 304.808);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);
}

void test_function_packPosition(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope(101.289, -16.724);
    telescope.timestamp = 0xefbeadde;

    uint8_t buffer[24];
    const uint8_t expected1[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};

    telescope.packPosition(buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, buffer, 24);

    // M13 for 10th August 1998 at 2310 hrs UT, for Birmingham UK (lat=52.5)
    telescope.setPosition(250.425, 36.466667);
    telescope.timestamp = 0xefbeadde;

    const uint8_t expected2[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x7A, 0x14, 0xB2, 0xDC, 0x8D, 0xEE, 0x19, 0x00, 0x00, 0x00, 0x00};

    telescope.packPosition(buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, buffer, 24);
}

void test_function_unpackPosition(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

    double ra, dec;
    uint64_t timestamp;
    bool res = false;
    res = telescope.unpackPosition(&ra, &dec, &timestamp, static_cast<uint8_t*>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(&ra, &dec, &timestamp, static_cast<uint8_t*>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, dec);    
}

void test_function_unpackPositionWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

    bool res = false;
    res = telescope.unpackPosition(static_cast<uint8_t*>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, telescope.position.dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(static_cast<uint8_t*>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, telescope.position.dec);    
}


void process(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_function_rad);
    RUN_TEST(test_function_deg);
    RUN_TEST(test_function_degToHours);
    RUN_TEST(test_function_fromHorizontalPosition);
    RUN_TEST(test_function_toHorizontalPosition);
    RUN_TEST(test_function_packPosition);
    RUN_TEST(test_function_unpackPosition);
    RUN_TEST(test_function_unpackPositionWrapper);
    UNITY_END();
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