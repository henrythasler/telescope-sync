#include <unity.h>
#include <alignment.h>
#include <telescope.h>

namespace Test_Alignment
{
    void test_function_addVertex(void)
    {
        Alignment alignment;
        TEST_ASSERT_EQUAL(0, alignment.getNumVertices());
        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        alignment.addVertexPair(Equatorial(1, 8), Equatorial());
        alignment.addVertexPair(Equatorial(4, 7), Equatorial());
        alignment.addVertexPair(Equatorial(10, 5), Equatorial());
        alignment.addVertexPair(Equatorial(5, 1), Equatorial());
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());

        // Duplicates should be removed
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());
    }

    void test_function_sorting(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Equatorial(10, 5), Equatorial());
        alignment.addVertexPair(Equatorial(5, 1), Equatorial());
        alignment.addVertexPair(Equatorial(4, 7), Equatorial());
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());
        alignment.addVertexPair(Equatorial(1, 8), Equatorial());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        VertexPair *vertices = alignment.getVerticesPtr();

        // test sorting
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[0].actual.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 8, vertices[0].actual.dec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, vertices[1].actual.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[1].actual.dec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 4, vertices[2].actual.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 7, vertices[2].actual.dec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[3].actual.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[3].actual.dec);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 10, vertices[4].actual.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[4].actual.dec);
    }

    void test_function_Triangulate1(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Equatorial(10, 5), Equatorial());
        alignment.addVertexPair(Equatorial(5, 1), Equatorial());
        alignment.addVertexPair(Equatorial(4, 7), Equatorial());
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());
        alignment.addVertexPair(Equatorial(1, 8), Equatorial());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        Triangle *triangles = alignment.getTrianglesPtr();
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
        alignment.addVertexPair(Equatorial(1, 8), Equatorial());
        alignment.addVertexPair(Equatorial(2, 1), Equatorial());
        alignment.addVertexPair(Equatorial(3, 1), Equatorial());
        alignment.addVertexPair(Equatorial(7, 8), Equatorial());
        alignment.addVertexPair(Equatorial(10, 6), Equatorial());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(3, alignment.getNumTriangles());

        Triangle *triangles = alignment.getTrianglesPtr();

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
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());
        alignment.addVertexPair(Equatorial(8, 8), Equatorial());
        alignment.addVertexPair(Equatorial(9, 2), Equatorial());
        alignment.addVertexPair(Equatorial(3, 5), Equatorial());
        alignment.addVertexPair(Equatorial(7, 2), Equatorial());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(2, alignment.getNumTriangles());

        Triangle *triangles = alignment.getTrianglesPtr();

        // compare with the *sorted* vertices
        TEST_ASSERT_EQUAL(1, triangles[0].p1);
        TEST_ASSERT_EQUAL(2, triangles[0].p2);
        TEST_ASSERT_EQUAL(3, triangles[0].p3);

        TEST_ASSERT_EQUAL(1, triangles[1].p1);
        TEST_ASSERT_EQUAL(0, triangles[1].p2);
        TEST_ASSERT_EQUAL(2, triangles[1].p3);
    }

    void test_function_TriangulateIncomplete(void)
    {
        Alignment alignment;

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix *matrices = alignment.getMatricesPtr();

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](2, 2));
    }

    void test_function_Triangulate1Point(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Equatorial(1, 2), Equatorial(4, 3));
        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix *matrices = alignment.getMatricesPtr();

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 3, matrices[0](0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](2, 2));
    }

    void test_function_Triangulate2Point(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Equatorial(1, 6), Equatorial(2, 6));
        alignment.addVertexPair(Equatorial(6, 8), Equatorial(5, 4));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix *matrices = alignment.getMatricesPtr();

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, .3793103, matrices[0](0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, -.5517241, matrices[0](1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, .5517241, matrices[0](0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, .3793103, matrices[0](1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrices[0](2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, -1.689655, matrices[0](0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 4.275862, matrices[0](1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrices[0](2, 2));        
    }

    void process(void)
    {
        UNITY_BEGIN();

        // Adding Vertices
        RUN_TEST(test_function_addVertex);
        RUN_TEST(test_function_sorting);

        // // Triangulation
        RUN_TEST(test_function_Triangulate1);
        RUN_TEST(test_function_Triangulate2);
        RUN_TEST(test_function_Triangulate3);
        RUN_TEST(test_function_TriangulateIncomplete);
        RUN_TEST(test_function_Triangulate1Point);
        RUN_TEST(test_function_Triangulate2Point);

        UNITY_END();
    }

} // Test_Mathhelper