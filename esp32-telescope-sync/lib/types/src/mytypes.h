#ifndef MYTYPES_H
#define MYTYPES_H

// #include <stdint.h>
// #include <stddef.h>
// #include <math.h>
// #include <algorithm>
// #include <helper.h>
#include <linalg.h>

struct Equatorial
{
    double ra = 0;  // in degrees
    double dec = 0; // in degrees
    Equatorial(double raIn, double decIn);
    Equatorial();
};

struct Horizontal
{
    double az = 0;  // in degrees
    double alt = 0; // in degrees
    Horizontal(double azIn, double altIn);
    Horizontal();
};

const double EPSILON = 0.000001;
struct Triangle
{
    int p1, p2, p3;
};

struct Edge
{
    int p1, p2;
};

struct Point
{
    double x = 0;
    double y = 0;
    Point(double xIn, double yIn);
    Point(Equatorial in);
    Point(Horizontal in);
    Point();    
};

struct VertexPair
{
    Horizontal actual;
    Horizontal reference;
};

enum TransformationType
{
    NONE = 0,
    POINT = 1,
    LINE = 2,
    TRIANGLE_OUT = 3,
    TRIANGLE_IN = 4,
};
#endif