#include <unity.h>
#include <alignment.h>

namespace Test_Alignment
{
    void test_function_addVertex(void)
    {
        Alignment alignment;
        TEST_ASSERT_EQUAL(0, alignment.getNumVertices());
        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        alignment.addVertex(1, 8);
        alignment.addVertex(4, 7);
        alignment.addVertex(10, 5);
        alignment.addVertex(5, 1);
        alignment.addVertex(3, 5);

        // Duplicates should be removed
        alignment.addVertex(3, 5);
        alignment.addVertex(3, 5);

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());
    }

    void test_function_sorting(void)
    {
        Alignment alignment;

        alignment.addVertex(10, 5);
        alignment.addVertex(5, 1);
        alignment.addVertex(4, 7);
        alignment.addVertex(3, 5);
        alignment.addVertex(1, 8);

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.Triangulate();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        Alignment::VertexPair *vertices = alignment.getVerticesPtr();

        // test sorting
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[0].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 8, vertices[0].y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, vertices[1].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[1].y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 4, vertices[2].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 7, vertices[2].y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[3].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[3].y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 10, vertices[4].x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[4].y);
    }

    void test_function_Triangulate1(void)
    {
        Alignment alignment;

        alignment.addVertex(10, 5);
        alignment.addVertex(5, 1);
        alignment.addVertex(4, 7);
        alignment.addVertex(3, 5);
        alignment.addVertex(1, 8);

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.Triangulate();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        Alignment::Triangle *triangles = alignment.getTrianglesPtr();
        // Alignment::VertexPair *vertices = alignment.getVerticesPtr();

        // for (int i = 0; i < alignment.getNumVertices(); i++)
        //     printf("Vertices %i: (%.0f, %.0f)\n", i,
        //            vertices[i].x, vertices[i].y);

        // printf("Triangles: %i\n", alignment.getNumTriangles());
        // for (int i = 0; i < alignment.getNumTriangles(); i++)
        //     printf("Triangle %i: (%.0f, %.0f) (%.0f, %.0f) (%.0f, %.0f)\n", i,
        //            vertices[triangles[i].p1].x, vertices[triangles[i].p1].y,
        //            vertices[triangles[i].p2].x, vertices[triangles[i].p2].y,
        //            vertices[triangles[i].p3].x, vertices[triangles[i].p3].y);

        // // compare with the *sorted* vertices
        TEST_ASSERT_EQUAL(3, triangles[0].p1);
        TEST_ASSERT_EQUAL(1, triangles[0].p2);
        TEST_ASSERT_EQUAL(4, triangles[0].p3);

        TEST_ASSERT_EQUAL(1, triangles[1].p1);
        TEST_ASSERT_EQUAL(0, triangles[1].p2);
        TEST_ASSERT_EQUAL(2, triangles[1].p3);

        TEST_ASSERT_EQUAL(1, triangles[2].p1);
        TEST_ASSERT_EQUAL(2, triangles[2].p2);
        TEST_ASSERT_EQUAL(4, triangles[2].p3);

        TEST_ASSERT_EQUAL(0, triangles[3].p1);
        TEST_ASSERT_EQUAL(1, triangles[3].p2);
        TEST_ASSERT_EQUAL(3, triangles[3].p3);
    }

    void test_function_Triangulate2(void)
    {
        Alignment alignment;

        // np.random.seed(19)
        alignment.addVertex(1, 8);
        alignment.addVertex(2, 1);
        alignment.addVertex(3, 1);
        alignment.addVertex(7, 8);
        alignment.addVertex(10, 6);

        alignment.Triangulate();

        TEST_ASSERT_EQUAL(3, alignment.getNumTriangles());

        Alignment::Triangle *triangles = alignment.getTrianglesPtr();

        // // compare with the *sorted* vertices
        TEST_ASSERT_EQUAL(2, triangles[0].p1);
        TEST_ASSERT_EQUAL(3, triangles[0].p2);
        TEST_ASSERT_EQUAL(4, triangles[0].p3);

        TEST_ASSERT_EQUAL(1, triangles[1].p1);
        TEST_ASSERT_EQUAL(0, triangles[1].p2);
        TEST_ASSERT_EQUAL(2, triangles[1].p3);

        TEST_ASSERT_EQUAL(2, triangles[2].p1);
        TEST_ASSERT_EQUAL(0, triangles[2].p2);
        TEST_ASSERT_EQUAL(3, triangles[2].p3);
    }

    void test_function_Triangulate3(void)
    {
        Alignment alignment;

        //     // np.random.seed(26)
        alignment.addVertex(3, 5);
        alignment.addVertex(8, 8);
        alignment.addVertex(9, 2);
        alignment.addVertex(3, 5);
        alignment.addVertex(7, 2);

        alignment.Triangulate();

        TEST_ASSERT_EQUAL(2, alignment.getNumTriangles());

        Alignment::Triangle *triangles = alignment.getTrianglesPtr();

        // compare with the *sorted* vertices
        TEST_ASSERT_EQUAL(1, triangles[0].p1);
        TEST_ASSERT_EQUAL(2, triangles[0].p2);
        TEST_ASSERT_EQUAL(3, triangles[0].p3);

        TEST_ASSERT_EQUAL(1, triangles[1].p1);
        TEST_ASSERT_EQUAL(0, triangles[1].p2);
        TEST_ASSERT_EQUAL(2, triangles[1].p3);
    }

    void process(void)
    {
        UNITY_BEGIN();

        // Adding Vertices
        RUN_TEST(test_function_addVertex);
        RUN_TEST(test_function_sorting);

        // Triangulation
        RUN_TEST(test_function_Triangulate1);
        RUN_TEST(test_function_Triangulate2);
        RUN_TEST(test_function_Triangulate3);

        UNITY_END();
    }

} // Test_Mathhelper