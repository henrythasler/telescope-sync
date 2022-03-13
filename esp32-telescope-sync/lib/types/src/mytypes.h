#pragma once

// #include <stdint.h>
// #include <stddef.h>
// #include <math.h>
// #include <algorithm>
// #include <helper.h>
// #include <linalg.h>

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

struct VertexPair
{
    Equatorial actual;
    Equatorial reference;
};