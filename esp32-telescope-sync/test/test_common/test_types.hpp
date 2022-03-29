#include <unity.h>
#include <mytypes.h>

namespace Test_Types
{

    void allTypes(void)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Point().x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 0, Point().y);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 1, Point(1,2).x);
        TEST_ASSERT_FLOAT_WITHIN(0.00001, 2, Point(1,2).y);
    }

    void process(void)
    {
        UNITY_BEGIN();
        RUN_TEST(allTypes);
        UNITY_END();
    }

} // Test_Mathhelper