#include <unity.h>
#include <nexstar.h>

namespace Test_Nexstar
{
    void test_function_nexstar_unknown(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss; // set initial position to enable operation before GNSS fix
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(0, bytes);
    }

    void test_function_nexstar_echo(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss; // set initial position to enable operation before GNSS fix
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "Ka";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(2, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("a#", response, bytes);
    }

    void test_function_nexstar_get_version(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss; // set initial position to enable operation before GNSS fix
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "V";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(3, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x04\x0A#", response, bytes);
    }

    void test_function_nexstar_get_location(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss(48, 11); // set initial position to enable operation before GNSS fix
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "w";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(9, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x30\x00\x00\x00\x0b\x00\x00\x00#", response, bytes);
    }

    void test_function_nexstar_get_deviceversion(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss; // set initial position to enable operation before GNSS fix
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "P\x01\x10\xfe\x00\x00\x00\x02";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(3, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x01\x00#", response, bytes);
    }

    void test_function_nexstar_get_time(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss; // set initial position to enable operation before GNSS fix
        gnss.utcTimestamp.tm_year = 2000;
        Telescope telescope;
        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "h";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(9, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x00\x00\x00\x00\x00\x00\x00\x00#", response, bytes);
    }        

    void test_function_nexstar_get_radec(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        // see http://jonvoisey.net/blog/2018/07/data-converting-alt-az-to-ra-dec-example/

        // Birmingham UK, 10th August 1998 at 2310
        GNSS gnss(52.5, -1.91667);
        gnss.utcTimestamp.tm_year = 1998;
        gnss.utcTimestamp.tm_mon = 8;
        gnss.utcTimestamp.tm_mday = 10;
        gnss.utcTimestamp.tm_hour = 23;
        gnss.utcTimestamp.tm_min = 10;
        gnss.utcTimestamp.tm_sec = 0;

        Telescope telescope;

        telescope.orientation.alt = 49.169122;
        telescope.orientation.az = 269.14634;

        // make sure the input is correct
        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(gnss.utcTimestamp, gnss.longitude);
        auto res = telescope.horizontalToEquatorial(269.14634, 49.169122, gnss.latitude, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, res.dec);

        NexStar nexstar(&telescope, &gnss);

        uint8_t sampleA[] = "e";
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        // printf("%s", (char *)response);
        TEST_ASSERT_EQUAL(18, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("B2148DFE,19EE8DF9#", response, bytes);
    }

    void test_function_nexstar_sync_precise_1(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss(48, 11);
        gnss.utcTimestamp.tm_year = 2022;
        gnss.utcTimestamp.tm_mon = 1;
        gnss.utcTimestamp.tm_mday = 13;
        gnss.utcTimestamp.tm_hour = 16;
        gnss.utcTimestamp.tm_min = 25;
        gnss.utcTimestamp.tm_sec = 9;

        Telescope telescope;
        telescope.setOrientation(0, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, telescope.offset.alt);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, telescope.offset.az);

        NexStar nexstar(&telescope, &gnss);

        // not yet aligned
        uint8_t isAlignmentComplete[] = "J";
        bytes = nexstar.handleRequest(isAlignmentComplete, sizeof(isAlignmentComplete), response, sizeof(response));
        TEST_ASSERT_EQUAL(2, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x00#", response, bytes);

        uint8_t sampleA[] = "s2E0F3189,0F32CD10"; // Moon
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(1, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("#", response, bytes);
        TEST_ASSERT_TRUE(telescope.isCalibrated);
        // TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentWritePointer);
        // TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentPoints);
        // TEST_ASSERT_FLOAT_WITHIN(0.01, 64.77, telescope.referencePoints[0].ra);
        // TEST_ASSERT_FLOAT_WITHIN(0.01, 21.37, telescope.referencePoints[0].dec);

        // TEST_ASSERT_FLOAT_WITHIN(0.01, 190.41, telescope.actualPoints[0].ra);
        // TEST_ASSERT_FLOAT_WITHIN(0.01, 42, telescope.actualPoints[0].dec);

        // should report aligned now
        bytes = nexstar.handleRequest(isAlignmentComplete, sizeof(isAlignmentComplete), response, sizeof(response));
        TEST_ASSERT_EQUAL(2, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x01#", response, bytes);
    }

    void test_function_nexstar_sync_precise_2(void)
    {
        uint8_t response[32];
        int32_t bytes = 0;

        GNSS gnss(48, 11);
        gnss.utcTimestamp.tm_year = 2022;
        gnss.utcTimestamp.tm_mon = 1;
        gnss.utcTimestamp.tm_mday = 18;
        gnss.utcTimestamp.tm_hour = 16;
        gnss.utcTimestamp.tm_min = 30;
        gnss.utcTimestamp.tm_sec = 32;

        Telescope telescope;
        telescope.setOrientation(0, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, telescope.offset.alt);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, telescope.offset.az);

        NexStar nexstar(&telescope, &gnss);

        // not yet aligned
        uint8_t isAlignmentComplete[] = "J";
        bytes = nexstar.handleRequest(isAlignmentComplete, sizeof(isAlignmentComplete), response, sizeof(response));
        TEST_ASSERT_EQUAL(2, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x00#", response, bytes);

        uint8_t sampleA[] = "sEF0EF519,F83084C1FFFFFF"; // Jupiter; add some garbage at the end to test limiter
        bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
        TEST_ASSERT_EQUAL(1, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("#", response, bytes);
        TEST_ASSERT_TRUE(telescope.isCalibrated);
        // TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentWritePointer);
        // TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentPoints);

        // should report aligned now
        bytes = nexstar.handleRequest(isAlignmentComplete, sizeof(isAlignmentComplete), response, sizeof(response));
        TEST_ASSERT_EQUAL(2, bytes);
        TEST_ASSERT_EQUAL_HEX8_ARRAY("\x01#", response, bytes);
    }

    void process(void)
    {
        UNITY_BEGIN();

        RUN_TEST(test_function_nexstar_unknown);
        RUN_TEST(test_function_nexstar_get_version);
        RUN_TEST(test_function_nexstar_get_location);
        RUN_TEST(test_function_nexstar_get_deviceversion);
        RUN_TEST(test_function_nexstar_get_time);
        RUN_TEST(test_function_nexstar_echo);
        RUN_TEST(test_function_nexstar_get_radec);
        RUN_TEST(test_function_nexstar_sync_precise_1);
        RUN_TEST(test_function_nexstar_sync_precise_2);

        UNITY_END();
    }
}