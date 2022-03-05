#include <unity.h>
#include <gnss.h>

namespace Test_GNSS
{
    void test_function_gnss_rmc(void)
    {
        GNSS gnss;
        bool res = false;

        string sentenceA = "$GPRMC,083055.00,A,4815.69961,N,01059.02625,E,2.158,,291221,,,A*79";
        res = gnss.fromRMC(sentenceA);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_TRUE_MESSAGE(gnss.valid, "valid");
        TEST_ASSERT_TRUE_MESSAGE(gnss.north, "north");
        TEST_ASSERT_TRUE_MESSAGE(gnss.east, "east");

        TEST_ASSERT_EQUAL_UINT32(8, gnss.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(30, gnss.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(55, gnss.utcTimestamp.tm_sec);
        TEST_ASSERT_EQUAL_UINT32(2021, gnss.utcTimestamp.tm_year);
        TEST_ASSERT_EQUAL_UINT32(12, gnss.utcTimestamp.tm_mon);
        TEST_ASSERT_EQUAL_UINT32(29, gnss.utcTimestamp.tm_mday);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 48.2616, gnss.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 10.9838, gnss.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1.1101, gnss.speed);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.course);

        string sentenceB = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62";
        res = gnss.fromRMC(sentenceB);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_TRUE_MESSAGE(gnss.valid, "valid");
        TEST_ASSERT_FALSE_MESSAGE(gnss.north, "north");
        TEST_ASSERT_TRUE_MESSAGE(gnss.east, "east");

        TEST_ASSERT_EQUAL_UINT32(8, gnss.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(18, gnss.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(36, gnss.utcTimestamp.tm_sec);
        TEST_ASSERT_EQUAL_UINT32(2098, gnss.utcTimestamp.tm_year);
        TEST_ASSERT_EQUAL_UINT32(9, gnss.utcTimestamp.tm_mon);
        TEST_ASSERT_EQUAL_UINT32(13, gnss.utcTimestamp.tm_mday);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, -37.8608, gnss.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 145.1227, gnss.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.speed);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 360, gnss.course);

        GNSS gnss2;
        string sentenceC = "$GPRMC,165011.00,V,,,,,,,291221,,,N*74";
        res = gnss2.fromRMC(sentenceC);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_FALSE_MESSAGE(gnss2.valid, "valid");
        TEST_ASSERT_FALSE_MESSAGE(gnss2.north, "north");
        TEST_ASSERT_FALSE_MESSAGE(gnss2.east, "east");

        TEST_ASSERT_EQUAL_UINT32(16, gnss2.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(50, gnss2.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(11, gnss2.utcTimestamp.tm_sec);
        TEST_ASSERT_EQUAL_UINT32(2021, gnss2.utcTimestamp.tm_year);
        TEST_ASSERT_EQUAL_UINT32(12, gnss2.utcTimestamp.tm_mon);
        TEST_ASSERT_EQUAL_UINT32(29, gnss2.utcTimestamp.tm_mday);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.speed);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.course);
    }

    void test_function_gnss_gga(void)
    {
        GNSS gnss;
        bool res = false;

        string sentenceA = "$GPGGA,083055.00,4815.69961,N,01059.02625,E,1,04,7.80,485.8,M,46.8,M,,*50";
        res = gnss.fromGGA(sentenceA);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_TRUE_MESSAGE(gnss.valid, "valid");
        TEST_ASSERT_TRUE_MESSAGE(gnss.north, "north");
        TEST_ASSERT_TRUE_MESSAGE(gnss.east, "east");

        TEST_ASSERT_EQUAL_UINT32(8, gnss.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(30, gnss.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(55, gnss.utcTimestamp.tm_sec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 48.2616, gnss.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 10.9838, gnss.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 485.8, gnss.altitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 7.8, gnss.dilution);

        TEST_ASSERT_EQUAL_UINT32(4, gnss.satUsed);

        string sentenceB = "$GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,19.7,M,,,,0000*1F";
        res = gnss.fromGGA(sentenceB);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_TRUE_MESSAGE(gnss.valid, "valid");
        TEST_ASSERT_FALSE_MESSAGE(gnss.north, "north");
        TEST_ASSERT_TRUE_MESSAGE(gnss.east, "east");

        TEST_ASSERT_EQUAL_UINT32(9, gnss.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(22, gnss.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(5, gnss.utcTimestamp.tm_sec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, -42.8426, gnss.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 147.3085, gnss.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 19.7, gnss.altitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 24.4, gnss.dilution);

        TEST_ASSERT_EQUAL_UINT32(4, gnss.satUsed);

        GNSS gnss2;
        string sentenceC = "$GPGGA,165011.00,,,,,0,00,99.99,,,,,,*64";
        res = gnss2.fromGGA(sentenceC);
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");

        TEST_ASSERT_FALSE_MESSAGE(gnss2.valid, "valid");
        TEST_ASSERT_FALSE_MESSAGE(gnss2.north, "north");
        TEST_ASSERT_FALSE_MESSAGE(gnss2.east, "east");

        TEST_ASSERT_EQUAL_UINT32(16, gnss2.utcTimestamp.tm_hour);
        TEST_ASSERT_EQUAL_UINT32(50, gnss2.utcTimestamp.tm_min);
        TEST_ASSERT_EQUAL_UINT32(11, gnss2.utcTimestamp.tm_sec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.latitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.longitude);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss2.altitude);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 99.99, gnss2.dilution);

        TEST_ASSERT_EQUAL_UINT32(0, gnss2.satUsed);
    }

    void test_function_gnss_gsv(void)
    {
        GNSS gnss;
        bool res = false;

        string sentenceA = "$GPGSV,3,1,12,02,11,234,15,07,54,060,14,08,03,065,,09,20,099,23*7C";
        res = gnss.fromGSV(sentenceA);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceA");
        TEST_ASSERT_EQUAL_UINT32(12, gnss.satView);

        string sentenceB = "$GPGSV,1,1,01,21,00,000,*4B";
        res = gnss.fromGSV(sentenceB);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceB");
        TEST_ASSERT_EQUAL_UINT32(1, gnss.satView);
    }

    void test_function_gnss_checksum(void)
    {
        GNSS gnss;
        bool res = false;

        string sentenceA = "$GPGSV,3,1,12,02,11,234,15,07,54,060,14,08,03,065,,09,20,099,23*7C";
        res = gnss.verifyChecksum(sentenceA);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceA");

        string sentenceB = "$GPGGA,165011.00,,,,,0,00,99.99,,,,,,*64";
        res = gnss.verifyChecksum(sentenceB);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceB");
    }

    void test_function_gnss_nmea(void)
    {
        GNSS gnss;
        bool res = false;

        string sentenceA = "$GPRMC,083055.00,A,4815.69961,N,01059.02625,E,2.158,,291221,,,A*79";
        res = gnss.fromNMEA(sentenceA);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceA");

        string sentenceB = "$GPRMC,082804.683,A,5205.9421,N,00506.4368,E,0.02,146.61,190408,,*0C";
        res = gnss.fromNMEA(sentenceB);
        TEST_ASSERT_TRUE_MESSAGE(res, "sentenceB");

        string sentenceC = "$NOTFOUND,082804.683,A,5205.9421,N,00506.4368,E,0.02,146.61,190408,,*0C";
        res = gnss.fromNMEA(sentenceC);
        TEST_ASSERT_FALSE_MESSAGE(res, "sentenceC");
    }

    void test_function_gnss_buffer(void)
    {
        GNSS gnss;
        uint32_t res = 0;

        uint8_t sampleA[] = "$GPRMC,083055.00,A,4815.69961,N,01059.02625,E,2.158,,291221,,,A*79\r\n";
        res = gnss.fromBuffer(sampleA, sizeof(sampleA));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, res, "sampleA");

        uint8_t sampleB[] = "$GPRMC,083055.00,A,4815.69961,N,01059.02625,E,2.158,,291221,,,A*79\r";
        res = gnss.fromBuffer(sampleB, sizeof(sampleB));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, res, "sampleB");

        uint8_t sampleC[] = "$GPRMC,083055.00,A,4815.69961,N,01059.02625,E,2.158,,291221,,,A*79\r\n$GPVTG,,T,,M,2.158,N,3.997,K,A*29\r\n$GPGGA,083055.00,4815.69961,N,01059.02625,E,1,04,7.80,485.8,M,46.8,M,,*50\r\n$GPGSA,A,3,02,07,30,06,,,,,,,,,14.68,7";
        res = gnss.fromBuffer(sampleC, sizeof(sampleC));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, res, "sampleC");
    }

    void process(void)
    {
        UNITY_BEGIN();

        RUN_TEST(test_function_gnss_checksum);

        // Low-Level decoder
        RUN_TEST(test_function_gnss_rmc);
        RUN_TEST(test_function_gnss_gga);
        RUN_TEST(test_function_gnss_gsv);

        // High-Level wrapper
        RUN_TEST(test_function_gnss_nmea);
        RUN_TEST(test_function_gnss_buffer);

        UNITY_END();
    }
}