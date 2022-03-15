#include <unity.h>
#include <helper.h>

namespace Test_Mathhelper
{

    void test_function_siderealtime_fmod(void)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.8, MathHelper::f_mod(-5.2, 2.));
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 40, MathHelper::f_mod(400, 360));
    }

    void test_function_siderealtime_julianday(void)
    {
        float res = 0;

        tm timestamp = {.tm_sec = 0, .tm_min = 0, .tm_hour = 0, .tm_mday = 1, .tm_mon = 1, .tm_year = 2000};
        res = MathHelper::julianDay(timestamp);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 2451544.5, res);

        timestamp = {.tm_sec = 0, .tm_min = 0, .tm_hour = 0, .tm_mday = 23, .tm_mon = 12, .tm_year = 2021};
        res = MathHelper::julianDay(timestamp);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 2459571.5, res);

        timestamp = {.tm_sec = 0, .tm_min = 0, .tm_hour = 0, .tm_mday = 2, .tm_mon = 1, .tm_year = 2022};
        res = MathHelper::julianDay(timestamp);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 2459581.5, res);

        timestamp = {.tm_sec = 0, .tm_min = 0, .tm_hour = 0, .tm_mday = 13, .tm_mon = 7, .tm_year = 2025};
        res = MathHelper::julianDay(timestamp);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 2460869.5, res);

        timestamp = {.tm_sec = 0, .tm_min = 0, .tm_hour = 22, .tm_mday = 1, .tm_mon = 12, .tm_year = 2006};
        res = MathHelper::julianDay(timestamp);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 2451544.5 + 2526, res);
    }

    void test_function_siderealtime_LST(void)
    {
        float res = 0;

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 10, .tm_hour = 23, .tm_mday = 10, .tm_mon = 8, .tm_year = 1998}, -1.9166667);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 304.808, res);

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 18, .tm_mday = 16, .tm_mon = 6, .tm_year = 1994}, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 174.7711, res);

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 34, .tm_min = 30, .tm_hour = 8, .tm_mday = 23, .tm_mon = 12, .tm_year = 2021}, -120);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 99.7504, res);

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 22, .tm_min = 13, .tm_hour = 6, .tm_mday = 13, .tm_mon = 7, .tm_year = 2025}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 35.7267, res);

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 1, .tm_mday = 2, .tm_mon = 1, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 127.6567, res);

        res = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 22, .tm_mday = 1, .tm_mon = 12, .tm_year = 2006}, 5);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 45.61655, res);
    }

    void test_function_amt_checksum(void)
    {
        TEST_ASSERT_TRUE(Checksum::verifyAmtCheckbits(0x61AB));
        TEST_ASSERT_FALSE(Checksum::verifyAmtCheckbits(0));
        TEST_ASSERT_FALSE(Checksum::verifyAmtCheckbits(0xffff));
    }

    void process(void)
    {
        UNITY_BEGIN();

        // sidereal time
        RUN_TEST(test_function_siderealtime_fmod);
        RUN_TEST(test_function_siderealtime_julianday);
        RUN_TEST(test_function_siderealtime_LST);

        // checksums
        RUN_TEST(test_function_amt_checksum);

        UNITY_END();
    }

} // Test_Mathhelper