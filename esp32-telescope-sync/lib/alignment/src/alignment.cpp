#include <alignment.h>

int XYZCompare(const void *v1, const void *v2)
{
    VertexPair *p1, *p2;

    p1 = (VertexPair *)v1;
    p2 = (VertexPair *)v2;
    if (p1->actual.ra < p2->actual.ra)
        return (-1);
    else if (p1->actual.ra > p2->actual.ra)
        return (1);
    else
        return (0);
}

Alignment::Alignment()
{
    this->maxVertices = MAX_ALIGNMENT_POINTS;
    this->vertices = new VertexPair[this->maxVertices + 3];
    this->triangles = new Triangle[3 * this->maxVertices];
    this->transormationMatrices = new BLA::Matrix<3, 3, BLA::Array<3, 3, double>>[3 * this->maxVertices];
}

bool Alignment::addVertexPair(Equatorial actual, Equatorial reference)
{
    // prevent adding duplicates
    for (int i = 0; i < this->numVertices; i++)
    {
        if ((this->vertices[i].actual.ra == actual.ra) && (this->vertices[i].actual.dec == actual.dec))
            return false;
    }

    // first vertex defines wrapping
    if (this->numVertices > 0)
    {
        if ((this->vertices[0].reference.ra - reference.ra) > 180)
        {
            reference.ra += 360;
        }

        if ((this->vertices[0].reference.ra - reference.ra) < -180)
        {
            reference.ra -= 360;
        }
    }
    // add new vertex
    this->vertices[this->numVertices].actual = actual;
    this->vertices[this->numVertices].reference = reference;
    this->numVertices = (this->numVertices + 1) % this->maxVertices;

    // make sure it's sorted correctly for triangulation
    qsort(this->vertices, this->numVertices, sizeof(VertexPair), XYZCompare);
    return true;
}

void Alignment::clearAll()
{
    this->numVertices = 0;
    this->numTriangles = 0;
}

int Alignment::getNumVertices()
{
    return this->numVertices;
}

int Alignment::getNumTriangles()
{
    return this->numTriangles;
}

Triangle *Alignment::getTrianglesPtr()
{
    return this->triangles;
}

TransformationMatrix *Alignment::getMatricesPtr()
{
    return this->transormationMatrices;
}

VertexPair *Alignment::getVerticesPtr()
{
    return this->vertices;
}

void Alignment::TriangulateActual()
{
    if (this->numVertices > 0)
    {
        this->TriangulateActual(this->numVertices, this->vertices, this->triangles, this->numTriangles);
    }
    this->updateTransformationMatrices();
}

// from http://paulbourke.net/papers/triangulate/
////////////////////////////////////////////////////////////////////////
// CircumCircle() :
//   Return true if a point (xp,yp) is inside the circumcircle made up
//   of the points (x1,y1), (x2,y2), (x3,y3)
//   The circumcircle centre is returned in (xc,yc) and the radius r
//   Note : A point on the edge is inside the circumcircle
////////////////////////////////////////////////////////////////////////

int Alignment::CircumCircle(double xp, double yp, double x1, double y1, double x2,
                            double y2, double x3, double y3, double &xc, double &yc, double &r)
{
    double m1, m2, mx1, mx2, my1, my2;
    double dx, dy, rsqr, drsqr;

    /* Check for coincident points */
    if (fabs(y1 - y2) < EPSILON && fabs(y2 - y3) < EPSILON)
        return (false);
    if (fabs(y2 - y1) < EPSILON)
    {
        m2 = -(x3 - x2) / (y3 - y2);
        mx2 = (x2 + x3) / 2.0;
        my2 = (y2 + y3) / 2.0;
        xc = (x2 + x1) / 2.0;
        yc = m2 * (xc - mx2) + my2;
    }
    else if (fabs(y3 - y2) < EPSILON)
    {
        m1 = -(x2 - x1) / (y2 - y1);
        mx1 = (x1 + x2) / 2.0;
        my1 = (y1 + y2) / 2.0;
        xc = (x3 + x2) / 2.0;
        yc = m1 * (xc - mx1) + my1;
    }
    else
    {
        m1 = -(x2 - x1) / (y2 - y1);
        m2 = -(x3 - x2) / (y3 - y2);
        mx1 = (x1 + x2) / 2.0;
        mx2 = (x2 + x3) / 2.0;
        my1 = (y1 + y2) / 2.0;
        my2 = (y2 + y3) / 2.0;
        xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
        yc = m1 * (xc - mx1) + my1;
    }
    dx = x2 - xc;
    dy = y2 - yc;
    rsqr = dx * dx + dy * dy;
    r = sqrt(rsqr);
    dx = xp - xc;
    dy = yp - yc;
    drsqr = dx * dx + dy * dy;
    return ((drsqr <= rsqr) ? true : false);
}
///////////////////////////////////////////////////////////////////////////////
// TriangulateActual() :
//   Triangulation subroutine
//   Takes as input NV vertices in array vertex
//   Returned is a list of ntri triangular faces in the array v
//   These triangles are arranged in a consistent clockwise order.
//   The triangle array 'v' should be malloced to 3 * nv
//   The vertex array vertex must be big enough to hold 3 more points
//   The vertex array must be sorted in increasing x values say
//
//   qsort(p,nv,sizeof(XYZ),XYZCompare);
///////////////////////////////////////////////////////////////////////////////

void Alignment::TriangulateActual(int nv, VertexPair vertex[], Triangle v[], int &ntri)
{
    int *complete = NULL;
    Edge *edges = NULL;
    Edge *p_EdgeTemp;
    int nedge = 0;
    int trimax, emax = 200;
    int inside;
    int i, j, k;
    double xp, yp, x1, y1, x2, y2, x3, y3, xc, yc, r;
    double xmin, xmax, ymin, ymax, xmid, ymid;
    double dx, dy, dmax;

    /* Allocate memory for the completeness list, flag for each triangle */
    trimax = 4 * nv;
    complete = new int[trimax];
    /* Allocate memory for the edge list */
    edges = new Edge[emax];
    /*
          Find the maximum and minimum vertex bounds.
          This is to allow calculation of the bounding triangle
    */
    xmin = vertex[0].actual.ra;
    ymin = vertex[0].actual.dec;
    xmax = xmin;
    ymax = ymin;
    for (i = 1; i < nv; i++)
    {
        if (vertex[i].actual.ra < xmin)
            xmin = vertex[i].actual.ra;
        if (vertex[i].actual.ra > xmax)
            xmax = vertex[i].actual.ra;
        if (vertex[i].actual.dec < ymin)
            ymin = vertex[i].actual.dec;
        if (vertex[i].actual.dec > ymax)
            ymax = vertex[i].actual.dec;
    }
    dx = xmax - xmin;
    dy = ymax - ymin;
    dmax = (dx > dy) ? dx : dy;
    xmid = (xmax + xmin) / 2.0;
    ymid = (ymax + ymin) / 2.0;
    /*
       Set up the supertriangle
       his is a triangle which encompasses all the sample points.
       The supertriangle coordinates are added to the end of the
       vertex list. The supertriangle is the first triangle in
       the triangle list.
    */
    vertex[nv + 0].actual.ra = xmid - 20 * dmax;
    vertex[nv + 0].actual.dec = ymid - dmax;
    vertex[nv + 1].actual.ra = xmid;
    vertex[nv + 1].actual.dec = ymid + 20 * dmax;
    vertex[nv + 2].actual.ra = xmid + 20 * dmax;
    vertex[nv + 2].actual.dec = ymid - dmax;
    v[0].p1 = nv;
    v[0].p2 = nv + 1;
    v[0].p3 = nv + 2;
    complete[0] = false;
    ntri = 1;
    /*
       Include each point one at a time into the existing mesh
    */
    for (i = 0; i < nv; i++)
    {
        xp = vertex[i].actual.ra;
        yp = vertex[i].actual.dec;
        nedge = 0;
        /*
             Set up the edge buffer.
             If the point (xp,yp) lies inside the circumcircle then the
             three edges of that triangle are added to the edge buffer
             and that triangle is removed.
        */
        for (j = 0; j < ntri; j++)
        {
            if (complete[j])
                continue;
            x1 = vertex[v[j].p1].actual.ra;
            y1 = vertex[v[j].p1].actual.dec;
            x2 = vertex[v[j].p2].actual.ra;
            y2 = vertex[v[j].p2].actual.dec;
            x3 = vertex[v[j].p3].actual.ra;
            y3 = vertex[v[j].p3].actual.dec;
            inside = CircumCircle(xp, yp, x1, y1, x2, y2, x3, y3, xc, yc, r);
            if (xc + r < xp)
                // Suggested
                // if (xc + r + EPSILON < xp)
                complete[j] = true;
            if (inside)
            {
                /* Check that we haven't exceeded the edge list size */
                if (nedge + 3 >= emax)
                {
                    emax += 100;
                    p_EdgeTemp = new Edge[emax];
                    for (int i = 0; i < nedge; i++)
                    { // Fix by John Bowman
                        p_EdgeTemp[i] = edges[i];
                    }
                    delete[] edges;
                    edges = p_EdgeTemp;
                }
                edges[nedge + 0].p1 = v[j].p1;
                edges[nedge + 0].p2 = v[j].p2;
                edges[nedge + 1].p1 = v[j].p2;
                edges[nedge + 1].p2 = v[j].p3;
                edges[nedge + 2].p1 = v[j].p3;
                edges[nedge + 2].p2 = v[j].p1;
                nedge += 3;
                v[j] = v[ntri - 1];
                complete[j] = complete[ntri - 1];
                ntri--;
                j--;
            }
        }
        /*
          Tag multiple edges
          Note: if all triangles are specified anticlockwise then all
          interior edges are opposite pointing in direction.
        */
        for (j = 0; j < nedge - 1; j++)
        {
            for (k = j + 1; k < nedge; k++)
            {
                if ((edges[j].p1 == edges[k].p2) && (edges[j].p2 == edges[k].p1))
                {
                    edges[j].p1 = -1;
                    edges[j].p2 = -1;
                    edges[k].p1 = -1;
                    edges[k].p2 = -1;
                }
                /* Shouldn't need the following, see note above */
                if ((edges[j].p1 == edges[k].p1) && (edges[j].p2 == edges[k].p2))
                {
                    edges[j].p1 = -1;
                    edges[j].p2 = -1;
                    edges[k].p1 = -1;
                    edges[k].p2 = -1;
                }
            }
        }
        /*
             Form new triangles for the current point
             Skipping over any tagged edges.
             All edges are arranged in clockwise order.
        */
        for (j = 0; j < nedge; j++)
        {
            if (edges[j].p1 < 0 || edges[j].p2 < 0)
                continue;
            v[ntri].p1 = edges[j].p1;
            v[ntri].p2 = edges[j].p2;
            v[ntri].p3 = i;
            complete[ntri] = false;
            ntri++;
        }
    }
    /*
          Remove triangles with supertriangle vertices
          These are triangles which have a vertex number greater than nv
    */
    for (i = 0; i < ntri; i++)
    {
        if (v[i].p1 >= nv || v[i].p2 >= nv || v[i].p3 >= nv)
        {
            v[i] = v[ntri - 1];
            ntri--;
            i--;
        }
    }
    delete[] edges;
    delete[] complete;
}

/**
 * ra = x
 * dec = y
 * */
TransformationMatrix Alignment::getTransformationMatrix(Equatorial actual)
{
    if ((this->numTriangles == 0) && (this->numVertices == 1))
    {
        return this->transormationMatrices[0];
    }
    else if ((this->numTriangles == 0) && (this->numVertices >= 2))
    {
        return this->transormationMatrices[0];
    }
    else if (this->numTriangles >= 1)
    {
        // need to find the triangle that contains the given actual position
        for (int i = 0; i < this->numTriangles; i++)
        {
            if (this->isInTriangle(Point(actual),
                                   Point(this->vertices[this->triangles[i].p1].actual),
                                   Point(this->vertices[this->triangles[i].p2].actual),
                                   Point(this->vertices[this->triangles[i].p3].actual)))
            {
                return this->transormationMatrices[i];
            }
        }
        // looks like the point is not in any of the triangles
        // find the triangle that is closest and use this transformation

        int triangleIndex = this->nearestTriangle(actual);
        if (triangleIndex >= 0)
        {
            return this->transormationMatrices[triangleIndex];
        }
    }

    // if nothing was found, default is identity matrix
    return BLA::Identity<3, 3>();
}

TransformationType Alignment::getTransformationType(Equatorial actual)
{
    if ((this->numTriangles == 0) && (this->numVertices == 1))
    {
        return TransformationType::POINT;
    }
    else if ((this->numTriangles == 0) && (this->numVertices >= 2))
    {
        return TransformationType::LINE;
    }
    else if (this->numTriangles >= 1)
    {
        // need to find the triangle that contains the given actual position
        for (int i = 0; i < this->numTriangles; i++)
        {
            if (this->isInTriangle(Point(actual),
                                   Point(this->vertices[this->triangles[i].p1].actual),
                                   Point(this->vertices[this->triangles[i].p2].actual),
                                   Point(this->vertices[this->triangles[i].p3].actual)))
            {
                return TransformationType::TRIANGLE_IN;
            }
        }
        // looks like the point is not in any of the triangles
        // find the triangle that is closest and use this transformation
        return TransformationType::TRIANGLE_OUT;
    }

    // if nothing was found, default is identity matrix
    return TransformationType::NONE;
}

void Alignment::updateTransformationMatrices(void)
{
    if ((this->numTriangles == 0) && (this->numVertices == 1))
    {
        this->transormationMatrices[0] = BLA::Identity<3, 3>();
        this->transormationMatrices[0](0, 2) = this->vertices[0].reference.ra - this->vertices[0].actual.ra;
        this->transormationMatrices[0](1, 2) = this->vertices[0].reference.dec - this->vertices[0].actual.dec;
        return;
    }
    else if ((this->numTriangles == 0) && (this->numVertices >= 2))
    {
        TransformationMatrix ref = {
            this->vertices[0].reference.ra, this->vertices[1].reference.ra, this->vertices[1].reference.ra + (this->vertices[1].reference.dec - this->vertices[0].reference.dec),
            this->vertices[0].reference.dec, this->vertices[1].reference.dec, this->vertices[1].reference.dec - (this->vertices[1].reference.ra - this->vertices[0].reference.ra),
            1, 1, 1};

        TransformationMatrix actual = {
            this->vertices[0].actual.ra, this->vertices[1].actual.ra, this->vertices[1].actual.ra + (this->vertices[1].actual.dec - this->vertices[0].actual.dec),
            this->vertices[0].actual.dec, this->vertices[1].actual.dec, this->vertices[1].actual.dec - (this->vertices[1].actual.ra - this->vertices[0].actual.ra),
            1, 1, 1};

        if (BLA::Invert(actual))
        {
            this->transormationMatrices[0] = ref * actual;
        }
        else
        {
            this->transormationMatrices[0] = BLA::Identity<3, 3>();
        }
    }
    else if (this->numTriangles >= 1)
    {
        for (int i = 0; i < this->numTriangles; i++)
        {
            TransformationMatrix ref = {
                this->vertices[this->triangles[i].p1].reference.ra, this->vertices[this->triangles[i].p2].reference.ra, this->vertices[this->triangles[i].p3].reference.ra,
                this->vertices[this->triangles[i].p1].reference.dec, this->vertices[this->triangles[i].p2].reference.dec, this->vertices[this->triangles[i].p3].reference.dec,
                1, 1, 1};

            TransformationMatrix actual = {
                this->vertices[this->triangles[i].p1].actual.ra, this->vertices[this->triangles[i].p2].actual.ra, this->vertices[this->triangles[i].p3].actual.ra,
                this->vertices[this->triangles[i].p1].actual.dec, this->vertices[this->triangles[i].p2].actual.dec, this->vertices[this->triangles[i].p3].actual.dec,
                1, 1, 1};

            if (BLA::Invert(actual))
            {
                this->transormationMatrices[i] = ref * actual;
            }
            else
            {
                this->transormationMatrices[i] = BLA::Identity<3, 3>();
            }
        }
    }
    else
    {
        this->transormationMatrices[0] = BLA::Identity<3, 3>();
    }
}

int Alignment::nearestTriangle(Equatorial actual)
{
    /*
            // Option 1: Switch to 1-Point alignment with nearest point
            float minDistance = INFINITY;
            int vertexIndex = -1;
            for (int i = 0; i < this->numVertices; i++)
            {
                float distance = (this->vertices[i].actual.ra - actual.ra) * (this->vertices[i].actual.ra - actual.ra) + (this->vertices[i].actual.dec - actual.dec) * (this->vertices[i].actual.dec - actual.dec);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    vertexIndex = i;
                }
            }
            TransformationMatrix matrix = BLA::Identity<3, 3>();
            matrix(0, 2) = this->vertices[vertexIndex].reference.ra - this->vertices[vertexIndex].actual.ra;
            matrix(1, 2) = this->vertices[vertexIndex].reference.dec - this->vertices[vertexIndex].actual.dec;
            return matrix;
    */

    // Option 2: Use transformationMatrix of nearest triangle
    float minDistance = INFINITY;
    int triangleIndex = -1;
    for (int i = 0; i < this->numTriangles; i++)
    {
        Equatorial center((this->vertices[this->triangles[i].p1].actual.ra +
                           this->vertices[this->triangles[i].p2].actual.ra +
                           this->vertices[this->triangles[i].p3].actual.ra) /
                              3.,
                          (this->vertices[this->triangles[i].p1].actual.dec +
                           this->vertices[this->triangles[i].p2].actual.dec +
                           this->vertices[this->triangles[i].p3].actual.dec) /
                              3.);
        float distance = (center.ra - actual.ra) * (center.ra - actual.ra) + (center.dec - actual.dec) * (center.dec - actual.dec);
        if (distance < minDistance)
        {
            minDistance = distance;
            triangleIndex = i;
        }
    }
    return triangleIndex;
}

Equatorial Alignment::getCalibratedOrientation(Equatorial actual)
{
    auto matrix = this->getTransformationMatrix(actual);
    // printf("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", matrix(0,0), matrix(0,1), matrix(0,2), matrix(1,0), matrix(1,1), matrix(1,2), matrix(2,0), matrix(2,1), matrix(2,2));
    Vector3D in = {actual.ra, actual.dec, 1};
    Vector3D out = matrix * in;
    // return (Equatorial(out(0), out(1)));
    return (Equatorial(MathHelper::f_mod(out(0), 360), out(1)));
}

double Alignment::triangleArea(Point p1, Point p2, Point p3)
{
    return fabs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2.0);
}

bool Alignment::isInTriangle(Point p, Point p1, Point p2, Point p3)
{
    float area = triangleArea(p1, p2, p3);
    float area1 = triangleArea(p, p2, p3);
    float area2 = triangleArea(p1, p, p3);
    float area3 = triangleArea(p1, p2, p);

    return (fabs(area - (area1 + area2 + area3)) < 0.0001);
}