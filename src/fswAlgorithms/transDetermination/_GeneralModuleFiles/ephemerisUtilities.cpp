#include "ephemerisUtilities.h"
/*
 * Function to evaluate a set of chebyshev polynomials (first argument) to a certain degree (second argument) at
 * a specific point (evaluationPoint)
 */
double calculateChebyValue(const double *coefficients,
                           const signed int numberOfCoefficients,
                           const double evaluationPoint) {
    double chebyPrev = 1.0;
    double chebyNow = evaluationPoint;
    const double valueMult = 2.0 * evaluationPoint;

    double estValue = coefficients[0] * chebyPrev;
    if (numberOfCoefficients <= 1) {
        return estValue;
    }
    estValue += coefficients[1] * chebyNow;
    for (int i = 2; i < numberOfCoefficients; ++i) {
        const double chebyLocalPrev = chebyNow;
        chebyNow = valueMult * chebyNow - chebyPrev;
        chebyPrev = chebyLocalPrev;
        estValue += coefficients[i] * chebyNow;
    }

    return estValue;
}
