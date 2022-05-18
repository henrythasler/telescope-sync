#include <mytypes.h>

Equatorial::Equatorial(double raIn, double decIn) : ra(raIn), dec(decIn){};
Equatorial::Equatorial() {}

Horizontal::Horizontal(double azIn, double altIn) : az(azIn), alt(altIn){};
Horizontal::Horizontal(Point in) : az(in.x), alt(in.y){};
Horizontal::Horizontal() {}

Point::Point(double xIn, double yIn) : x(xIn), y(yIn){};
Point::Point(Equatorial in) : x(in.ra), y(in.dec){};
Point::Point(Horizontal in) : x(in.az), y(in.alt){};
Point::Point() {}
