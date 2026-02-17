// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "safeMath.h"

#include <math.h>

double safeAcos(double x) {
    if (x < -1.0)
        return acos(-1);
    else if (x > 1.0)
        return acos(1);
    return acos(x);
}

double safeAsin(double x) {
    if (x < -1.0)
        return asin(-1);
    else if (x > 1.0)
        return asin(1);
    return asin(x);
}

double safeSqrt(double x) {
    if (x < 0.0) return 0.0;
    return sqrt(x);
}
