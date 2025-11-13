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
