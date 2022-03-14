#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include <linalg.h>
#include <mytypes.h>

#define MAX_ALIGNMENT_POINTS (64)

typedef BLA::Matrix<3, 3, BLA::Array<3, 3, double>> TransformationMatrix;

class Alignment
{
public:
    Alignment();
    void TriangulateActual();

    bool addVertexPair(Equatorial actual, Equatorial reference);
    int getNumVertices();
    int getNumTriangles();
    Triangle *getTrianglesPtr();
    VertexPair *getVerticesPtr();
    TransformationMatrix *getMatricesPtr();

    Equatorial getCalibratedOrientation(Equatorial actual);

private:
    int CircumCircle(double, double, double, double, double, double, double, double, double &, double &, double &);
    void TriangulateActual(int nv, VertexPair vertex[], Triangle v[], int &ntri);
    void updateTransformationMatrices(void);

    TransformationMatrix getTransformationMatrix(Equatorial actual);

    int maxVertices = 0;

    int numVertices = 0;
    VertexPair *vertices = NULL;

    int numTriangles = 0;
    Triangle *triangles = NULL;

    // each triangle has a transformation matrix assigned, that transforms the actual vertex to the reference vertex for each VertexPair
    TransformationMatrix *transormationMatrices = NULL;  
};
#endif // ALIGNMENT_H