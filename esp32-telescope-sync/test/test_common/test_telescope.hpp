#include <unity.h>
#include <telescope.h>

#ifndef ARDUINO
#include <stdio.h>
#include <chrono>
#define PI M_PI
#define HALF_PI M_PI_2
#define TWO_PI (2 * M_PI)
#endif

namespace Test_Telescope
{
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

        res = telescope.horizontalToEquatorial(Horizontal(269.14634, 49.169122), 52.5, 304.80762);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, res.dec);

        telescope.horizontalToEquatorial(Horizontal(269.14634, 49.169122), 52.5, 304.80762, &res);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, res.dec);
    }

    void test_function_toHorizontalPosition(void)
    {
        Telescope telescope;
        Horizontal res = telescope.equatorialToHorizontal(250.425, 36.467, 52.5, 304.808);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);

        res = telescope.equatorialToHorizontal(250.425, 36.467, 52.5, 304.808);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);

        res = telescope.equatorialToHorizontal(Equatorial(250.425, 36.467), 52.5, 304.808);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);

        telescope.equatorialToHorizontal(Equatorial(250.425, 36.467), 52.5, 304.808, &res);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);
    }

    void test_function_packPosition(void)
    {
        // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
        Telescope telescope;
        Equatorial position;
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

    void test_function_packPositionNegative(void)
    {
        Equatorial position(0, 0);
        Telescope telescope;
        uint32_t res = -1;
        uint8_t buffer[23];

        // insufficient buffer size
        res = telescope.packPosition(position, 0, buffer, sizeof(buffer));
        TEST_ASSERT_EQUAL_UINT32(0, res);
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
        Equatorial position;
        uint64_t timestamp;

        // LENGTH too small
        uint8_t dataA[] = {0x13, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};
        res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataA), sizeof(dataA));
        TEST_ASSERT_FALSE(res);

        // data too short
        uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B};
        res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataB), sizeof(dataB));
        TEST_ASSERT_FALSE(res);

        // type != 0
        uint8_t dataC[] = {0x14, 0x00, 0x01, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};
        res = telescope.unpackPosition(&position, &timestamp, static_cast<uint8_t *>(dataC), sizeof(dataC));
        TEST_ASSERT_FALSE(res);
    }

    void test_function_unpackPositionWrapper(void)
    {
        // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
        Telescope telescope;
        Equatorial position;
        uint64_t timestamp;
        bool res = false;

        uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};
        res = telescope.unpackPosition(&position.ra, &position.dec, &timestamp, static_cast<uint8_t *>(dataA), sizeof(dataA));
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");
        TEST_ASSERT_FLOAT_WITHIN(0.001, 101.289, position.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.001, -16.724 + 360.0, position.dec);

        // Betelgeuse on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
        uint8_t dataB[] = {0x14, 0x00, 0x00, 0x00, 0xE5, 0xE3, 0xA5, 0xCD, 0x21, 0xD4, 0x05, 0x00, 0x58, 0x59, 0x25, 0x3F, 0x12, 0x66, 0x44, 0x05};

        res = telescope.unpackPosition(&position.ra, &position.dec, &timestamp, static_cast<uint8_t *>(dataB), sizeof(dataB));
        TEST_ASSERT_TRUE_MESSAGE(res, "return value");
        TEST_ASSERT_FLOAT_WITHIN(0.001, 88.79891, position.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.001, 7.4070, position.dec);
    }

    void test_function_unpackPositionAnotherWrapper(void)
    {
        // Sirius on 2021-12-28 00:38:39 UTC+1 at lng=11 lat=48
        Telescope telescope;
        uint8_t dataA[] = {0x14, 0x00, 0x00, 0x00, 0x9C, 0x78, 0x7A, 0x74, 0x21, 0xD4, 0x05, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3A, 0x7D, 0x1B, 0xF4};

        Equatorial position;
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

    void test_function_getCalibratedOrientationIdentity(void)
    {
        Telescope telescope;

        auto orientation = telescope.equatorialToHorizontal(1, 2, 0, 0);
        telescope.setOrientation(orientation);

        auto res = telescope.getCalibratedOrientation(0, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.001, 2, res.dec);
        auto hor = telescope.equatorialToHorizontal(res, 0, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.001, orientation.az, hor.az);
        TEST_ASSERT_FLOAT_WITHIN(0.001, orientation.alt, hor.alt);

        // check time invariance
        auto res2 = telescope.getCalibratedOrientation(0, 30);
        auto hor2 = telescope.equatorialToHorizontal(res2, 0, 30);
        TEST_ASSERT_FLOAT_WITHIN(0.001, orientation.az, hor2.az);
        TEST_ASSERT_FLOAT_WITHIN(0.001, orientation.alt, hor2.alt);
    }

    void test_function_getCalibratedOrientation1Point(void)
    {
        // Testsite, 11°E, 48°N, height 0m
        // 2022-05-15 20:00:00 UTC
        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 20, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 184.5282, localSiderealTimeDegrees);

        // Arcturus
        telescope.setOrientation(Horizontal(110, 60));
        telescope.addReferencePoint(Equatorial(213.91, 19.17), 48, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(0, telescope.alignment.getNumTriangles());

        auto res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 213.91, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        auto corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);

        // Check time invariance
        double localSiderealTimeDegrees2 = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 6, .tm_hour = 22, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 216.1144, localSiderealTimeDegrees2);

        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 245.49, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);
    }

    void test_function_getCalibratedOrientation2Point(void)
    {
        // Testsite, 11°E, 48°N, height 0m
        // 2022-05-15 20:00:00 UTC
        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 20, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 184.5282, localSiderealTimeDegrees);

        // Arcturus
        telescope.setOrientation(Horizontal(110, 60));
        telescope.addReferencePoint(Equatorial(213.91, 19.17), 48, localSiderealTimeDegrees);

        // Izar
        telescope.setOrientation(Horizontal(90, 64));
        telescope.addReferencePoint(Equatorial(221.25, 27.07), 48, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(2, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(0, telescope.alignment.getNumTriangles());

        // Arcturus
        telescope.setOrientation(Horizontal(110, 60));
        auto res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 213.91, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        auto corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);

        // Check time invariance
        double localSiderealTimeDegrees2 = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 6, .tm_hour = 22, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 216.1144, localSiderealTimeDegrees2);

        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 245.49, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);
    }

    void test_function_getCalibratedOrientation3PointSampleA(void)
    {
        // Testsite, 11°E, 48°N, height 0m
        // 2022-05-15 20:00:00 UTC
        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 20, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 184.5282, localSiderealTimeDegrees);

        // Arcturus
        telescope.setOrientation(Horizontal(110, 60));
        telescope.addReferencePoint(Equatorial(213.91, 19.17), 48, localSiderealTimeDegrees);

        // Izar
        telescope.setOrientation(Horizontal(90, 64));
        telescope.addReferencePoint(Equatorial(221.25, 27.07), 48, localSiderealTimeDegrees);

        // 25 Boo
        telescope.setOrientation(Horizontal(90, 67));
        telescope.addReferencePoint(Equatorial(217.96, 30.37), 48, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(3, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumTriangles());

        // Arcturus
        telescope.setOrientation(Horizontal(110, 60));
        auto res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 213.91, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        auto corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);

        // Check time invariance
        double localSiderealTimeDegrees2 = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 6, .tm_hour = 22, .tm_mday = 15, .tm_mon = 5, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 216.1144, localSiderealTimeDegrees2);

        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 245.49, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 19.17, res.dec);

        corrected = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees2);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 130.22, corrected.az);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 52.63, corrected.alt);

    }

    void test_function_getCalibratedOrientation3PointSampleB(void)
    {
        // Testsite, 11°E, 48°N, height 0m
        // 2022-03-16 18:00:00 UTC
        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 18, .tm_mday = 16, .tm_mon = 3, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 95.3072, localSiderealTimeDegrees);

        // we map the actual position triangle Sirius, Rigel and Betelgeuse
        // with (az: 0, alt:0) set to Sirius

        // Sirius
        telescope.setOrientation(Horizontal(0, 0));
        telescope.addReferencePoint(Equatorial(101.29, -16.72), 48, localSiderealTimeDegrees);

        // Rigel
        telescope.setOrientation(Horizontal(199.25 - 173.42, 32.01 - 25.05));
        telescope.addReferencePoint(Equatorial(78.63, -8.20), 48, localSiderealTimeDegrees);

        // Betelgeuse
        telescope.setOrientation(Horizontal(189.43 - 173.42, 49.08 - 25.05));
        telescope.addReferencePoint(Equatorial(88.79, 7.40), 48, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(3, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumTriangles());

        // Test all 3 points first
        telescope.setOrientation(Horizontal(0, 0));
        auto res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 101.29, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -16.72, res.dec);

        telescope.setOrientation(Horizontal(199.25 - 173.42, 32.01 - 25.05));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 78.63, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -8.20, res.dec);

        telescope.setOrientation(Horizontal(189.43 - 173.42, 49.08 - 25.05));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 88.79, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 7.40, res.dec);

        // and something in between (Alnitak)
        telescope.setOrientation(Horizontal(192.76 - 173.42, 39.35 - 25.05));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 85.17, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -1.95, res.dec);

        // and something outside the triangle (Bellantrix)
        telescope.setOrientation(Horizontal(200 - 173.42, 47 - 25.05));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 81.41, res.ra); // this is way off, but it's out of the calibration area, so what.
        TEST_ASSERT_FLOAT_WITHIN(0.01, 6.54, res.dec);

        // check time invariance (Rigel)
        // telescope.setOrientation(Horizontal(199.25 - 173.42, 32.01 - 25.05));
        // double localSiderealTimeDegrees2 = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 18, .tm_mday = 16, .tm_mon = 3, .tm_year = 2022}, 11);
        // TEST_ASSERT_FLOAT_WITHIN(0.0001, 216.1144, localSiderealTimeDegrees2);

        // res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees + 30);
        // auto hor = telescope.equatorialToHorizontal(res, 48, localSiderealTimeDegrees + 30);
        // TEST_ASSERT_FLOAT_WITHIN(0.01, 199.25 - 173.42, hor.az);
        // TEST_ASSERT_FLOAT_WITHIN(0.01, 32.01 - 25.05, hor.alt);
    }

    void test_function_realWorldExample1(void)
    {
        // Date: 2022-04-06
        // Time: 19:30 UTC

        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 30, .tm_hour = 19, .tm_mday = 6, .tm_mon = 4, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 138.5674, localSiderealTimeDegrees);

        // Betelgeuse
        telescope.setOrientation(Horizontal(280.64, 33.51));
        telescope.addReferencePoint(Equatorial(88.79, 7.4), 48.3, localSiderealTimeDegrees);

        // Sirius
        telescope.setOrientation(Horizontal(256.7, 18.15));
        telescope.addReferencePoint(Equatorial(101.28, -16.72), 48.3, localSiderealTimeDegrees);

        // Procyon
        telescope.setOrientation(Horizontal(252.2, 44.19));
        telescope.addReferencePoint(Equatorial(114.82, 5.22), 48.3, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(3, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumTriangles());

        // Test all 3 points first
        telescope.setOrientation(Horizontal(280.64, 33.51));
        auto res = telescope.getCalibratedOrientation(48.3, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 88.79, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 7.4, res.dec);

        telescope.setOrientation(Horizontal(256.7, 18.15));
        res = telescope.getCalibratedOrientation(48.3, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 101.28, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -16.72, res.dec);

        telescope.setOrientation(Horizontal(252.2, 44.19));
        res = telescope.getCalibratedOrientation(48.3, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 114.82, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 5.22, res.dec);

        // and something in between
        telescope.setOrientation(Horizontal(261, 30));
        res = telescope.getCalibratedOrientation(48.3, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 102.40, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -4.55, res.dec);

        // auto matrix = telescope.alignment.getTransformationMatrix(telescope.horizontalToEquatorial(telescope.orientation, 48.3, localSiderealTimeDegrees));
        // printf("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        //        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        //        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        //        matrix(2, 0), matrix(2, 1), matrix(2, 2));
        // expected  [ 0.91,  -0.32,  38.73,  -4.96,  -13.83,  761.51,  0,  0,  1]
        // actual    [ 0.9 ,  -0.31,  39.95,  -4.89,  -13.85,  755.12,  0,  0,  1]
    }

    void test_function_wrapAlignment1(void)
    {
        // telescope positions are fake
        // Date: 2022-04-09
        // Time: 08:52 UTC

        Telescope telescope;

        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 52, .tm_hour = 8, .tm_mday = 9, .tm_mon = 4, .tm_year = 2022}, 11);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 341.5876, localSiderealTimeDegrees);

        // Markab
        telescope.setOrientation(Horizontal(342, 60));
        telescope.addReferencePoint(Equatorial(346, 15), 48, localSiderealTimeDegrees);

        // Algenib
        telescope.setOrientation(Horizontal(308, 56));
        telescope.addReferencePoint(Equatorial(3 + 360, 15), 48, localSiderealTimeDegrees);

        // Alpheratz
        telescope.setOrientation(Horizontal(308, 70));
        telescope.addReferencePoint(Equatorial(2 + 360, 29), 48, localSiderealTimeDegrees);

        TEST_ASSERT_EQUAL(3, telescope.alignment.getNumVertices());
        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumTriangles());

        // Test all 3 points first
        telescope.setOrientation(Horizontal(342, 60));
        auto res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 346, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 15, res.dec);

        telescope.setOrientation(Horizontal(308, 56));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 3, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 15, res.dec);

        telescope.setOrientation(Horizontal(308, 70));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 2, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 29, res.dec);

        // and something in between
        telescope.setOrientation(Horizontal(320, 60));
        res = telescope.getCalibratedOrientation(48, localSiderealTimeDegrees);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 357.25, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 16.96, res.dec);

        // auto matrix = telescope.alignment.getTransformationMatrix(telescope.horizontalToEquatorial(telescope.orientation, 48, localSiderealTimeDegrees));
        // printf("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        //        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        //        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        //        matrix(2, 0), matrix(2, 1), matrix(2, 2));
        // expected  [ 0.91,  -0.32,  38.73,  -4.96,  -13.83,  761.51,  0,  0,  1]
        // actual    [ 0.9 ,  -0.31,  39.95,  -4.89,  -13.85,  755.12,  0,  0,  1]
    }

    void test_function_addReferencePoint(void)
    {
        Telescope telescope(180, 60.34);

        telescope.addReferencePoint(Equatorial(101.29, -16.72), 38.59, 297.93);
        TEST_ASSERT_EQUAL(1, telescope.alignment.getNumVertices());
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
        RUN_TEST(test_function_packPositionNegative);
        RUN_TEST(test_function_unpackPosition);
        RUN_TEST(test_function_unpackPositionWrapper);
        RUN_TEST(test_function_unpackPositionAnotherWrapper);
        RUN_TEST(test_function_unpackPositionNegative);

        // n-Point-Alignment
        RUN_TEST(test_function_addReferencePoint);
        RUN_TEST(test_function_getCalibratedOrientationIdentity);
        RUN_TEST(test_function_getCalibratedOrientation1Point);
        RUN_TEST(test_function_getCalibratedOrientation2Point);
        RUN_TEST(test_function_getCalibratedOrientation3PointSampleA);
        RUN_TEST(test_function_getCalibratedOrientation3PointSampleB);

        RUN_TEST(test_function_realWorldExample1);
        RUN_TEST(test_function_wrapAlignment1);

        UNITY_END();
    }
}