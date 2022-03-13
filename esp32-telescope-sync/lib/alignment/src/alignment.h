#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <mytypes.h>

#define MAX_ALIGNMENT_POINTS (64)

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

private:
    int CircumCircle(double, double, double, double, double, double, double, double, double &, double &, double &);
    void TriangulateActual(int nv, VertexPair vertex[], Triangle v[], int &ntri);

    int maxVertices = 0;

    int numVertices = 0;
    VertexPair *vertices = NULL;

    int numTriangles = 0;
    Triangle *triangles = NULL;
};
#endif // ALIGNMENT_H