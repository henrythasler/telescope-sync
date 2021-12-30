#include <unity.h>
#include <telescope.h>
#include <gnss.h>
#include <siderealtime.h>

#ifndef ARDUINO
#include <stdio.h>
#include <chrono>
#define PI M_PI
#define HALF_PI M_PI_2
#define TWO_PI (2 * M_PI)
#endif

void test_function_rad(void)
{
    Telescope telescope;
    TEST_ASSERT_EQUAL_FLOAT(HALF_PI, telescope.rad(90));
    TEST_ASSERT_EQUAL_FLOAT(PI, telescope.rad(180));
    TEST_ASSERT_EQUAL_FLOAT(TWO_PI, telescope.rad(360));

    TEST_ASSERT_EQUAL_FLOAT(-HALF_PI, telescope.rad(-90));
    TEST_ASSERT_EQUAL_FLOAT(-PI, telescope.rad(-180));
    TEST_ASSERT_EQUAL_FLOAT(-TWO_PI, telescope.rad(-360));
}

void test_function_deg(void)
{
    Telescope telescope;
    TEST_ASSERT_EQUAL_FLOAT(90, telescope.deg(HALF_PI));
    TEST_ASSERT_EQUAL_FLOAT(180, telescope.deg(PI));
    TEST_ASSERT_EQUAL_FLOAT(360, telescope.deg(TWO_PI));

    TEST_ASSERT_EQUAL_FLOAT(-90, telescope.deg(-HALF_PI));
    TEST_ASSERT_EQUAL_FLOAT(-180, telescope.deg(-PI));
    TEST_ASSERT_EQUAL_FLOAT(-360, telescope.deg(-TWO_PI));
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
    Telescope telescope;
    double ra = 101.289;
    double dec = -16.724;
    uint64_t timestamp = 0;

    uint8_t buffer[24];
    const uint8_t expected[24] = {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};

    uint32_t res = 0;
    res = telescope.packPosition(&ra, &dec, &timestamp, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 24);
}

void test_function_packPositionWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope(101.289, -16.724);
    telescope.timestamp = 0xefbeadde;
    uint32_t res = 0;

    uint8_t buffer[24];
    const uint8_t expected1[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};

    res = telescope.packPosition(buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, buffer, 24);

    // M13 for 10th August 1998 at 2310 hrs UT, for Birmingham UK (lat=52.5)
    telescope.setPosition(250.425, 36.466667);
    telescope.timestamp = 0xefbeadde;

    const uint8_t expected2[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x7A, 0x14, 0xB2, 0xDC, 0x8D, 0xEE, 0x19, 0x00, 0x00, 0x00, 0x00};

    res = telescope.packPosition(buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, buffer, 24);
}

void test_function_packPositionAnotherWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    Telescope::Equatorial position;
    position.ra = 101.289;
    position.dec = -16.724;
    uint64_t timestamp = 0;

    uint8_t buffer[24];
    const uint8_t expected[24] = {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};

    uint32_t res = 0;
    res = telescope.packPosition(&position, &timestamp, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, 24);
}

void test_function_unpackPosition(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

    double ra, dec;
    uint64_t timestamp;
    bool res = false;
    res = telescope.unpackPosition(&ra, &dec, &timestamp, static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(&ra, &dec, &timestamp, static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, dec);
}

void test_function_unpackPositionNegative(void)
{
    Telescope telescope;
    bool res = true;

    // LENGTH too small
    uint8_t dataA[] = {0x13, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};
    res = telescope.unpackPosition(static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_FALSE(res);

    // data too short
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B};
    res = telescope.unpackPosition(static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_FALSE(res);
}

void test_function_unpackPositionWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

    bool res = false;
    res = telescope.unpackPosition(static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, telescope.position.dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, telescope.position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, telescope.position.dec);
}

void test_function_unpackPositionAnotherWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

    Telescope::Equatorial position;
    bool res = false;
    res = telescope.unpackPosition(&position, NULL, static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, position.dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(&position, NULL, static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, position.dec);
}

void test_function_calibrate(void)
{
    Telescope telescope(30, 50);
    Telescope::Equatorial reference;
    reference.ra = 10;
    reference.dec = 20;
    telescope.calibrate(&reference);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -20, telescope.offset.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -30, telescope.offset.dec);

    Telescope::Equatorial corrected;
    telescope.getCalibratedPosition(&corrected);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 10, corrected.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 20, corrected.dec);
}

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

    string sentenceB = "$GPRMC,082804.683,A,5205.9421,N,00506.4368,E,0.02,146.61,190408,,*0C";
    res = gnss.fromRMC(sentenceB);
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");

    TEST_ASSERT_TRUE_MESSAGE(gnss.valid, "valid");
    TEST_ASSERT_TRUE_MESSAGE(gnss.north, "north");
    TEST_ASSERT_TRUE_MESSAGE(gnss.east, "east");

    TEST_ASSERT_EQUAL_UINT32(8, gnss.utcTimestamp.tm_hour);
    TEST_ASSERT_EQUAL_UINT32(28, gnss.utcTimestamp.tm_min);
    TEST_ASSERT_EQUAL_UINT32(4, gnss.utcTimestamp.tm_sec);
    TEST_ASSERT_EQUAL_UINT32(2008, gnss.utcTimestamp.tm_year);
    TEST_ASSERT_EQUAL_UINT32(4, gnss.utcTimestamp.tm_mon);
    TEST_ASSERT_EQUAL_UINT32(19, gnss.utcTimestamp.tm_mday);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 52.0990, gnss.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 5.1073, gnss.longitude);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.0103, gnss.speed);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 146.61, gnss.course);


    string sentenceC = "$GPRMC,165011.00,V,,,,,,,291221,,,N*74";
    res = gnss.fromRMC(sentenceC);
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");

    TEST_ASSERT_FALSE_MESSAGE(gnss.valid, "valid");
    TEST_ASSERT_FALSE_MESSAGE(gnss.north, "north");
    TEST_ASSERT_FALSE_MESSAGE(gnss.east, "east");

    TEST_ASSERT_EQUAL_UINT32(16, gnss.utcTimestamp.tm_hour);
    TEST_ASSERT_EQUAL_UINT32(50, gnss.utcTimestamp.tm_min);
    TEST_ASSERT_EQUAL_UINT32(11, gnss.utcTimestamp.tm_sec);
    TEST_ASSERT_EQUAL_UINT32(2021, gnss.utcTimestamp.tm_year);
    TEST_ASSERT_EQUAL_UINT32(12, gnss.utcTimestamp.tm_mon);
    TEST_ASSERT_EQUAL_UINT32(29, gnss.utcTimestamp.tm_mday);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.longitude);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.speed);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.course);    
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


    string sentenceC = "$GPGGA,165011.00,,,,,0,00,99.99,,,,,,*64";
    res = gnss.fromGGA(sentenceC);
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");

    TEST_ASSERT_FALSE_MESSAGE(gnss.valid, "valid");
    TEST_ASSERT_FALSE_MESSAGE(gnss.north, "north");
    TEST_ASSERT_FALSE_MESSAGE(gnss.east, "east");

    TEST_ASSERT_EQUAL_UINT32(16, gnss.utcTimestamp.tm_hour);
    TEST_ASSERT_EQUAL_UINT32(50, gnss.utcTimestamp.tm_min);
    TEST_ASSERT_EQUAL_UINT32(11, gnss.utcTimestamp.tm_sec);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.longitude);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, gnss.altitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 99.99, gnss.dilution);

    TEST_ASSERT_EQUAL_UINT32(0, gnss.satUsed);
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

void test_function_siderealtime_julianday(void)
{
    SiderealTime siderealTime;
    float res = 0;

    tm timestamp {.tm_mday=1, .tm_mon=1, .tm_year=2000};
    res = siderealTime.julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2451544.5, res);

    timestamp = {.tm_mday=23, .tm_mon=12, .tm_year=2021};
    res = siderealTime.julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2459571.5, res);

    timestamp = {.tm_mday=13, .tm_mon=7, .tm_year=2025};
    res = siderealTime.julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2460869.5, res);
}

void process(void)
{
    UNITY_BEGIN();
    // Telescope Class
    RUN_TEST(test_function_rad);
    RUN_TEST(test_function_deg);
    RUN_TEST(test_function_degToHours);
    RUN_TEST(test_function_fromHorizontalPosition);
    RUN_TEST(test_function_toHorizontalPosition);
    RUN_TEST(test_function_packPosition);
    RUN_TEST(test_function_packPositionWrapper);
    RUN_TEST(test_function_packPositionAnotherWrapper);
    RUN_TEST(test_function_unpackPosition);
    RUN_TEST(test_function_unpackPositionWrapper);
    RUN_TEST(test_function_unpackPositionAnotherWrapper);
    RUN_TEST(test_function_unpackPositionNegative);
    RUN_TEST(test_function_calibrate);

    // GNSS Class
    RUN_TEST(test_function_gnss_checksum);
    // Low-Level decoder
    RUN_TEST(test_function_gnss_rmc);
    RUN_TEST(test_function_gnss_gga);
    RUN_TEST(test_function_gnss_gsv);
    // High-Level wrapper
    RUN_TEST(test_function_gnss_nmea);
    RUN_TEST(test_function_gnss_buffer);

    // sidereal time
    RUN_TEST(test_function_siderealtime_julianday);

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