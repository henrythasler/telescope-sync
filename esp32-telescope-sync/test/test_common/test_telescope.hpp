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

    // void test_function_calibrate(void)
    // {
    //     Telescope telescope;
    //     Telescope::Equatorial reference;
    //     reference.ra = 101.29;
    //     reference.dec = -16.72;
    //     telescope.setOrientation(180, 45);
    //     double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees({.tm_sec = 0, .tm_min = 0, .tm_hour = 1, .tm_mday = 2, .tm_mon = 1, .tm_year = 2022}, 11);
    //     TEST_ASSERT_FLOAT_WITHIN(0.0001, 127.6567, localSiderealTimeDegrees);

    //     telescope.calibrate(reference, 48, localSiderealTimeDegrees);
    //     TEST_ASSERT_FLOAT_WITHIN(0.01, -23.88, telescope.offset.alt);
    //     TEST_ASSERT_FLOAT_WITHIN(0.01, 27.13, telescope.offset.az);
    // }

    void test_function_addReferencePoint(void)
    {
        Telescope telescope;
        Telescope::Equatorial reference;
        reference.ra = 101.29;
        reference.dec = -16.72;

        telescope.setOrientation(180, 60.34);

        for (int i = 0; i < 400; i++)
            telescope.addReferencePoint(&reference, 38.59, 297.93);

        TEST_ASSERT_EQUAL_INT32(16, telescope.alignmentWritePointer);
        TEST_ASSERT_EQUAL_INT32(64, telescope.alignmentPoints);

        TEST_ASSERT_FLOAT_WITHIN(0.01, 101.29, telescope.referencePoints[0].ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, -16.72, telescope.referencePoints[0].dec);

        TEST_ASSERT_FLOAT_WITHIN(0.01, 297.92, telescope.actualPoints[0].ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 8.93, telescope.actualPoints[0].dec);
    }

    void test_function_getTransformationMatrix1Point(void)
    {
        Telescope telescope;
        Telescope::Equatorial reference;

        reference.ra = 4;
        reference.dec = 3;

        auto orientation = telescope.equatorialToHorizontal(1, 2, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentWritePointer);
        TEST_ASSERT_EQUAL_INT32(1, telescope.alignmentPoints);

        TEST_ASSERT_FLOAT_WITHIN(0.01, 4, telescope.referencePoints[0].ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 3, telescope.referencePoints[0].dec);

        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, 1, telescope.actualPoints[0].ra, "telescope.actualPoints[0].ra");
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, 2, telescope.actualPoints[0].dec, "telescope.actualPoints[0].dec");

        auto res = telescope.getTransformationMatrix(0);
        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, res(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, res(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, res(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, res(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 3, res(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res(2, 2));
    }

    void test_function_getTransformationMatrix2Point(void)
    {
        Telescope telescope;
        Telescope::Equatorial reference;

        reference.ra = 2;
        reference.dec = 6;
        auto orientation = telescope.equatorialToHorizontal(1, 6, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        reference.ra = 5;
        reference.dec = 4;
        orientation = telescope.equatorialToHorizontal(6, 8, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        TEST_ASSERT_EQUAL_INT32(2, telescope.alignmentWritePointer);
        TEST_ASSERT_EQUAL_INT32(2, telescope.alignmentPoints);

        auto res = telescope.getTransformationMatrix(0);
        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, .3793103, res(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -.5517241, res(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, res(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, .5517241, res(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, .3793103, res(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, res(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -1.689655, res(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 4.275862, res(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1, res(2, 2));
    }

    void test_function_getTransformationMatrix3Point(void)
    {
        Telescope telescope;
        Telescope::Equatorial reference;

        reference.ra = 2;
        reference.dec = 1;
        auto orientation = telescope.equatorialToHorizontal(1, 2, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        reference.ra = 8;
        reference.dec = 2;
        orientation = telescope.equatorialToHorizontal(6, 3, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        reference.ra = 3;
        reference.dec = 4;
        orientation = telescope.equatorialToHorizontal(3, 6, 0, 0);
        telescope.setOrientation(orientation);
        telescope.addReferencePoint(&reference, 0, 0);

        TEST_ASSERT_EQUAL_INT32(3, telescope.alignmentWritePointer);
        TEST_ASSERT_EQUAL_INT32(3, telescope.alignmentPoints);

        auto res = telescope.getTransformationMatrix(0);
        // printf("%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", res(0,0), res(0,1), res(0,2), res(1,0), res(1,1), res(1,2), res(2,0), res(2,1), res(2,2));
        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.27777778, res(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.05555556, res(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, res(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.38888889, res(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.72222222, res(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, res(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.5, res(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.5, res(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1, res(2, 2));
    }

    void test_function_calibrateMatrix(void)
    {
        Telescope telescope;

        auto orientation = telescope.equatorialToHorizontal(4, 8, 0, 0);
        telescope.setOrientation(orientation);

        BLA::Matrix<3, 3, BLA::Array<3, 3, double>> mat = {1.27777778, -0.38888889, 1.5, 0.05555556, 0.72222222, -0.5, 0., 0., 1.};
        auto res = telescope.getCalibratedOrientation(mat, 0, 0);

        TEST_ASSERT_FLOAT_WITHIN(0.01, 3.5, res.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 5.5, res.dec);
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
        RUN_TEST(test_function_unpackPositionAnotherWrapper);
        RUN_TEST(test_function_unpackPositionNegative);
        // RUN_TEST(test_function_calibrate);

        // n-Point-Alignment
        RUN_TEST(test_function_addReferencePoint);
        RUN_TEST(test_function_getTransformationMatrix1Point);
        RUN_TEST(test_function_getTransformationMatrix2Point);
        RUN_TEST(test_function_getTransformationMatrix3Point);
        RUN_TEST(test_function_calibrateMatrix);

        UNITY_END();
    }
}