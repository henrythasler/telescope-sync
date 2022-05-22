#include <unity.h>
#include <mytypes.h>

namespace Test_Types
{

    void allTypes(void)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Point().x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Point().y);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Point(1, 2).x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Point(1, 2).y);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Point(Equatorial(1, 2)).x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Point(Equatorial(1, 2)).y);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Point(Horizontal(1, 2)).x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Point(Horizontal(1, 2)).y);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Horizontal().az);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Horizontal().alt);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Equatorial().ra);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Equatorial().dec);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Horizontal(1, 2).az);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Horizontal(1, 2).alt);

        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Horizontal(Point(1, 2)).az);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Horizontal(Point(1, 2)).alt);
    }

    void process(void)
    {
        UNITY_BEGIN();
        RUN_TEST(allTypes);
        UNITY_END();
    }

} // Test_Mathhelper