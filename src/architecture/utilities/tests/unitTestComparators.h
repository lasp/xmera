// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef UNITTESTCOMPARATORS_H
#define UNITTESTCOMPARATORS_H

#include <math.h>

int isEqual(double a, double b, double accuracy) {
    if (fabs(a - b) > accuracy) {
        return 0;
    }
    return 1;
}

int isEqualRel(double a, double b, double accuracy) {
    if (fabs(a - b) / fabs(a) > accuracy) {
        return 0;
    }
    return 1;
}

#endif  // UNITTESTCOMPARATORS_H
