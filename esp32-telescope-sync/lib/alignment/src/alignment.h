#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include <linalg.h>
#include <mytypes.h>
#include <helper.h>

#define MAX_ALIGNMENT_POINTS (64)

typedef BLA::Matrix<3, 3, BLA::Array<3, 3, double>> TransformationMatrix;
typedef BLA::Matrix<3, 1, BLA::Array<3, 1, double>> Vector3D;

class Alignment
{
public:
    Alignment();
    void TriangulateActual();

    void clearAll();
    bool addVertexPair(Point actual, Point reference);
    int getNumVertices();
    int getNumTriangles();
    Triangle *getTrianglesPtr();
    VertexPair *getVerticesPtr();
    TransformationMatrix *getMatricesPtr();

    double triangleArea(Point p1, Point p2, Point p3);
    bool isInTriangle(Point p, Point p1, Point p2, Point p3);
    int nearestTriangle(Point actual);

    Point getCalibratedOrientation(Point actual);
    TransformationMatrix getTransformationMatrix(Point actual);
    TransformationType getTransformationType(Point actual);

private:
    int CircumCircle(double, double, double, double, double, double, double, double, double &, double &, double &);
    void TriangulateActual(int nv, VertexPair vertex[], Triangle v[], int &ntri);
    void updateTransformationMatrices(void);

    int maxVertices = 0;

    int numVertices = 0;
    VertexPair *vertices = NULL;

    int numTriangles = 0;
    Triangle *triangles = NULL;

    // each triangle has a transformation matrix assigned, that transforms the actual vertex to the reference vertex for each VertexPair
    TransformationMatrix *transormationMatrices = NULL;
};
#endif // ALIGNMENT_H