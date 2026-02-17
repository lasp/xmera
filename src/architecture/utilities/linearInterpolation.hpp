// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LINEARINTERPOLATION_H
#define LINEARINTERPOLATION_H

#include <cassert>

/*! This function uses linear interpolation to solve for the value of an unknown function of a single variables f(x)
 at the point x.
@return double
@param x1 Data point x1
@param x2 Data point x2
@param y1 Function value at point x1
@param y2 Function value at point x2
@param x Function x coordinate for interpolation
*/
double linearInterpolation(double x1, double x2, double y1, double y2, double x) {
    assert(x1 < x && x < x2);

    return y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1);
}

#endif  // LINEARINTERPOLATION_H
