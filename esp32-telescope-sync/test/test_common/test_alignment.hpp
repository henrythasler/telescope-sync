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

        alignment.addVertexPair(Horizontal(1, 8), Horizontal());
        alignment.addVertexPair(Horizontal(4, 7), Horizontal());
        alignment.addVertexPair(Horizontal(10, 5), Horizontal());
        alignment.addVertexPair(Horizontal(5, 1), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());

        // Duplicates should be removed
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());
    }

    void test_function_sorting(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(10, 5), Horizontal());
        alignment.addVertexPair(Horizontal(5, 1), Horizontal());
        alignment.addVertexPair(Horizontal(4, 7), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(1, 8), Horizontal());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        VertexPair *vertices = alignment.getVerticesPtr();

        // test sorting
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[0].actual.x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 8, vertices[0].actual.y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, vertices[1].actual.x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[1].actual.y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 4, vertices[2].actual.x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 7, vertices[2].actual.y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[3].actual.x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, vertices[3].actual.y);

        TEST_ASSERT_FLOAT_WITHIN(0.0001, 10, vertices[4].actual.x);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, vertices[4].actual.y);
    }

    void test_function_Triangulate1(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(10, 5), Horizontal());
        alignment.addVertexPair(Horizontal(5, 1), Horizontal());
        alignment.addVertexPair(Horizontal(4, 7), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(1, 8), Horizontal());

        TEST_ASSERT_EQUAL(5, alignment.getNumVertices());

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        Triangle *triangles = alignment.getTrianglesPtr();

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
        alignment.addVertexPair(Horizontal(1, 8), Horizontal());
        alignment.addVertexPair(Horizontal(2, 1), Horizontal());
        alignment.addVertexPair(Horizontal(3, 1), Horizontal());
        alignment.addVertexPair(Horizontal(7, 8), Horizontal());
        alignment.addVertexPair(Horizontal(10, 6), Horizontal());

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
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(8, 8), Horizontal());
        alignment.addVertexPair(Horizontal(9, 2), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(7, 2), Horizontal());

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

    void test_function_TriangulateInvalid(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 1), Horizontal(1, 1));
        alignment.addVertexPair(Horizontal(2, 2), Horizontal(2, 2));
        alignment.addVertexPair(Horizontal(3, 3), Horizontal(3, 3));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(0, 0));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(2, 2));
    }

    void test_function_Triangulate1Point(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 2), Horizontal(4, 3));
        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(0, 0));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 3, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(2, 2));

        // test transformation
        auto res = alignment.getCalibratedOrientation(Horizontal(4, 8));
        TEST_ASSERT_FLOAT_WITHIN(0.01, 7, res.x);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 9, res.y);
    }

    void test_function_Triangulate2Point(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 6), Horizontal(2, 6));
        alignment.addVertexPair(Horizontal(6, 8), Horizontal(5, 4));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(0, 0));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, .3793103, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, -.5517241, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, .5517241, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, .3793103, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, -1.689655, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 4.275862, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(2, 2));

        // test transformation
        auto res = alignment.getCalibratedOrientation(Horizontal(4, 8));
        TEST_ASSERT_FLOAT_WITHIN(0.01, 4.24137931, res.x);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 5.10344828, res.y);
    }

    void test_function_Triangulate3Point(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 2), Horizontal(2, 1));
        alignment.addVertexPair(Horizontal(6, 3), Horizontal(8, 2));
        alignment.addVertexPair(Horizontal(3, 6), Horizontal(3, 4));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(1, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(3, 4));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.27777778, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.05555556, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.38888889, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.72222222, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.5, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.5, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1, matrix(2, 2));

        // test transformation
        auto res = alignment.getCalibratedOrientation(Horizontal(3, 4));
        TEST_ASSERT_FLOAT_WITHIN(0.01, 3.77777778, res.x);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 2.55555556, res.y);
    }

    void test_function_Triangulate3PointOutside1(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 2), Horizontal(2, 1));
        alignment.addVertexPair(Horizontal(6, 3), Horizontal(8, 2));
        alignment.addVertexPair(Horizontal(3, 6), Horizontal(3, 4));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(1, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(3, 4));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.27777778, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.05555556, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.38888889, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0.72222222, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1.5, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, -0.5, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, 1, matrix(2, 2));

        // test transformation
        auto res = alignment.getCalibratedOrientation(Horizontal(8, 8));
        TEST_ASSERT_FLOAT_WITHIN(0.01, 8.61111111, res.x);
        TEST_ASSERT_FLOAT_WITHIN(0.01, 5.72222222, res.y);
    }

    void test_function_getCalibratedOrientationIdentity(void)
    {
        Alignment alignment;
        alignment.TriangulateActual();
        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());
        auto res = alignment.getCalibratedOrientation(Horizontal(1, 2));

        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, res.x);
        TEST_ASSERT_FLOAT_WITHIN(0.001, 2, res.y);
    }

    void test_function_ClearAlignment(void)
    {
        Alignment alignment;

        alignment.addVertexPair(Horizontal(1, 2), Horizontal(2, 1));
        alignment.addVertexPair(Horizontal(6, 3), Horizontal(8, 2));
        alignment.addVertexPair(Horizontal(3, 6), Horizontal(3, 4));

        alignment.TriangulateActual();

        TEST_ASSERT_EQUAL(3, alignment.getNumVertices());
        TEST_ASSERT_EQUAL(1, alignment.getNumTriangles());

        alignment.clearAll();

        alignment.TriangulateActual();
        TEST_ASSERT_EQUAL(0, alignment.getNumVertices());
        TEST_ASSERT_EQUAL(0, alignment.getNumTriangles());

        TransformationMatrix matrix = alignment.getTransformationMatrix(Horizontal(3, 4));

        // 1st column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(0, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(1, 0));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 0));

        // 2nd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(0, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(1, 1));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(2, 1));

        // 3rd column
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(0, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 0, matrix(1, 2));
        TEST_ASSERT_FLOAT_WITHIN(0.001, 1, matrix(2, 2));        
    }

    void test_function_triangleArea(void)
    {
        Alignment alignment;
        double res = 0;

        Point A(0, 0);
        Point B(3, 0);
        Point C(3, 4);
        Point D(-3, -4);

        res = alignment.triangleArea(A, B, C);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 6, res);

        res = alignment.triangleArea(A, B, D);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 6, res);

        res = alignment.triangleArea(B, C, D);
        TEST_ASSERT_FLOAT_WITHIN(0.0001, 12, res);
    }

    void test_function_isInTriangle(void)
    {
        Alignment alignment;

        Point A(0, 0);
        Point B(3, 0);
        Point C(3, 4);
        Point D(-3, -4);

        TEST_ASSERT_TRUE(alignment.isInTriangle(Point(1, 1), A, B, C));
        TEST_ASSERT_FALSE(alignment.isInTriangle(Point(1, 2), A, B, C));
        TEST_ASSERT_TRUE(alignment.isInTriangle(Point(0, 0), A, B, C));
        TEST_ASSERT_TRUE(alignment.isInTriangle(Point(-1.4, -2.8), B, C, D));
        TEST_ASSERT_FALSE(alignment.isInTriangle(Point(-1.4, -3), B, C, D));
    }

    void test_function_nearestTriangle(void)
    {
        Alignment alignment;

        // np.random.seed(7)
        alignment.addVertexPair(Horizontal(10, 5), Horizontal());
        alignment.addVertexPair(Horizontal(5, 1), Horizontal());
        alignment.addVertexPair(Horizontal(4, 7), Horizontal());
        alignment.addVertexPair(Horizontal(3, 5), Horizontal());
        alignment.addVertexPair(Horizontal(1, 8), Horizontal());

        alignment.TriangulateActual();

        // Triangle *triangles = alignment.getTrianglesPtr();
        // VertexPair *vertices = alignment.getVerticesPtr();

        // for (int i = 0; i < alignment.getNumVertices(); i++)
        //     printf("Vertices %i: (%.0f, %.0f)\n", i,
        //            vertices[i].actual.x, vertices[i].actual.y);

        // printf("Triangles: %i\n", alignment.getNumTriangles());
        // for (int i = 0; i < alignment.getNumTriangles(); i++)
        //     printf("Triangle %i: (%.0f, %.0f) (%.0f, %.0f) (%.0f, %.0f)\n", i,
        //            vertices[triangles[i].p1].actual.x, vertices[triangles[i].p1].actual.y,
        //            vertices[triangles[i].p2].actual.x, vertices[triangles[i].p2].actual.y,
        //            vertices[triangles[i].p3].actual.x, vertices[triangles[i].p3].actual.y);

        TEST_ASSERT_EQUAL(4, alignment.getNumTriangles());

        TEST_ASSERT_EQUAL(2, alignment.nearestTriangle(Horizontal(7, 7)));
        TEST_ASSERT_EQUAL(0, alignment.nearestTriangle(Horizontal(5, 0)));
        TEST_ASSERT_EQUAL(1, alignment.nearestTriangle(Horizontal(0, 10)));
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
        RUN_TEST(test_function_TriangulateIncomplete);
        RUN_TEST(test_function_TriangulateInvalid);
        RUN_TEST(test_function_Triangulate1Point);
        RUN_TEST(test_function_Triangulate2Point);
        RUN_TEST(test_function_Triangulate3Point);

        RUN_TEST(test_function_Triangulate3PointOutside1);

        // Triangle Math
        RUN_TEST(test_function_triangleArea);
        RUN_TEST(test_function_isInTriangle);
        RUN_TEST(test_function_nearestTriangle);

        // Calibration
        RUN_TEST(test_function_getCalibratedOrientationIdentity);
        RUN_TEST(test_function_ClearAlignment);

        UNITY_END();
    }

} // Test_Mathhelper