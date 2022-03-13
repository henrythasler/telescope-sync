#ifndef ALIGNMENT_H
#define ALIGNMENT_H

using namespace std;

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include "telescope.h"

#define MAX_ALIGNMENT_POINTS (64)

class Alignment
{
public:
    Alignment();

    const double EPSILON = 0.000001;
    struct Triangle
    {
        int p1, p2, p3;
    };

    struct Edge
    {
        int p1, p2;
    };

    struct VertexPair
    {
        Telescope::Equatorial actual;
        Telescope::Equatorial reference;
        // double x, y, z;
    };

    void TriangulateActual();

    bool addVertex(double x, double y);
    int getNumVertices();
    int getNumTriangles();
    Alignment::Triangle* getTrianglesPtr();
    Alignment::VertexPair* getVerticesPtr();

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