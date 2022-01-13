#include <unity.h>
#include <telescope.h>
#include <gnss.h>
#include <nexstar.h>
#include <helper.h>

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
    TEST_ASSERT_EQUAL_FLOAT(22, telescope.degToHours(-30));
}

void test_function_fromHorizontalPosition(void)
{
    Telescope telescope;
    auto res = telescope.horizontalToEquatorial(180, 60.34, 38.59, 297.93);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 297.92, res.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 8.93, res.dec);

    res = telescope.horizontalToEquatorial(269.14634, 49.169122, 52.5, 304.80762);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, res.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, res.dec);
}

void test_function_toHorizontalPosition(void)
{
    Telescope telescope;
    Telescope::Horizontal res = telescope.equatorialToHorizontal(250.425, 36.467, 52.5, 304.808);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);

    res = telescope.equatorialToHorizontal(250.425, 36.467, 52.5, 304.808);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);

    Telescope::Equatorial pos;
    pos.ra = 250.425;
    pos.dec = 36.467;
    res = telescope.equatorialToHorizontal(pos, 52.5, 304.808);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);
}

void test_function_packPosition(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    Telescope::Equatorial position;
    uint8_t buffer[24];
    uint32_t res = 0;

    const uint8_t expected1[24] = {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};
    res = telescope.packPosition(101.289, -16.724, 0, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, buffer, 24);

    const uint8_t expected2[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};
    res = telescope.packPosition(101.289, -16.724, 0xefbeadde, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, buffer, 24);

    position.ra = 250.425;
    position.dec = 36.466667;
    const uint8_t expected3[24] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x7A, 0x14, 0xB2, 0xDC, 0x8D, 0xEE, 0x19, 0x00, 0x00, 0x00, 0x00};
    res = telescope.packPosition(position, 0xefbeadde, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, buffer, 24);

    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    position.ra = 101.289;
    position.dec = -16.724;
    const uint8_t expected4[24] = {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x19, 0x07, 0x48, 0x1C, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};
    res = telescope.packPosition(position, 0, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(24, res);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected4, buffer, 24);
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
    Telescope::Equatorial position;
    uint64_t timestamp;

    // LENGTH too small
    uint8_t dataA[] = {0x13, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};
    res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_FALSE(res);

    // data too short
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B};
    res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_FALSE(res);
}

void test_function_unpackPositionWrapper(void)
{
    // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    Telescope telescope;
    Telescope::Equatorial position;
    uint64_t timestamp;
    bool res = false;

    uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};
    res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataA), sizeof(dataA));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, position.dec);

    // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
    uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

    res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataB), sizeof(dataB));
    TEST_ASSERT_TRUE_MESSAGE(res, "return value");
    TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, position.ra);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, position.dec);
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
    Telescope telescope;
    Telescope::Equatorial reference;
    reference.ra = 101.29;
    reference.dec = -16.72;
    telescope.setOrientation(45, 180);
    double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 1, .tm_mday = 2, .tm_mon = 1, .tm_year = 2022}, 11);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 127.6567, localSiderealTimeDegrees);

    telescope.calibrate(reference, 48, localSiderealTimeDegrees);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -23.88, telescope.offset.alt);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 27.13, telescope.offset.az);
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

void test_function_siderealtime_fmod(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.8, MathHelper::f_mod(-5.2, 2.));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 40, MathHelper::f_mod(400, 360));
}

void test_function_siderealtime_julianday(void)
{
    float res = 0;

    tm timestamp{.tm_mday = 1, .tm_mon = 1, .tm_year = 2000};
    res = MathHelper::julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2451544.5, res);

    timestamp = {.tm_mday = 23, .tm_mon = 12, .tm_year = 2021};
    res = MathHelper::julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2459571.5, res);

    timestamp = {.tm_mday = 2, .tm_mon = 1, .tm_year = 2022};
    res = MathHelper::julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2459581.5, res);

    timestamp = {.tm_mday = 13, .tm_mon = 7, .tm_year = 2025};
    res = MathHelper::julianDay(timestamp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 2460869.5, res);
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

    telescope.orientation.az = 269.14634;
    telescope.orientation.alt = 49.169122;

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

void test_function_nexstar_sync_precise(void)
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

    uint8_t sampleA[] = "s2E0F3189,0F32CD10";   // Moon
    bytes = nexstar.handleRequest(sampleA, sizeof(sampleA), response, sizeof(response));
    TEST_ASSERT_EQUAL(1, bytes);
    TEST_ASSERT_EQUAL_HEX8_ARRAY("#", response, bytes);
    TEST_ASSERT_TRUE(telescope.isCalibrated);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 39.34, telescope.offset.alt);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 101.90, telescope.offset.az);

    // should report aligned now
    bytes = nexstar.handleRequest(isAlignmentComplete, sizeof(isAlignmentComplete), response, sizeof(response));
    TEST_ASSERT_EQUAL(2, bytes);
    TEST_ASSERT_EQUAL_HEX8_ARRAY("\x01#", response, bytes);
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
    RUN_TEST(test_function_siderealtime_fmod);
    RUN_TEST(test_function_siderealtime_julianday);
    RUN_TEST(test_function_siderealtime_LST);

    // nexstar
    RUN_TEST(test_function_nexstar_echo);
    RUN_TEST(test_function_nexstar_get_radec);
    RUN_TEST(test_function_nexstar_sync_precise);

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