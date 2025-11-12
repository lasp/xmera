#ifndef BILINEARINTERPOLATION_H
#define BILINEARINTERPOLATION_H

#include <cassert>

/*! This function uses bilinear interpolation to solve for the value of an unknown function of two variables f(x,y)
 at the point (x,y).
@return double
@param x1 Data point x1
@param x2 Data point x2
@param y1 Data point y1
@param y2 Data point y2
@param z11 Function value at point (x1, y1)
@param z12 Function value at point (x1, y2)
@param z21 Function value at point (x2, y1)
@param z22 Function value at point (x2, y2)
@param x Function x coordinate for interpolation
@param y Function y coordinate for interpolation
*/
double bilinearInterpolation(double x1,
                             double x2,
                             double y1,
                             double y2,
                             double z11,
                             double z12,
                             double z21,
                             double z22,
                             double x,
                             double y) {

    assert(x1 < x && x < x2);
    assert(y1 < y && y < y2);

    return 1 / ((x2 - x1) * (y2 - y1)) * (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y)
                                          + z12 * (x2 - x) * (y - y1)
                                          + z22 * (x - x1) * (y - y1));
}

#endif // BILINEARINTERPOLATION_H
