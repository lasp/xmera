#ifndef _EPHEMERIS_UTILITIES_H_
#define _EPHEMERIS_UTILITIES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/*! Calculate Chebychev Polynominal */
double calculateChebyValue(const double *chebyCoeff, const signed int nCoeff, const double evalValue);

#ifdef __cplusplus
}
#endif

#endif
