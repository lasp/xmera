#include "svIntegratorEuler.h"

svIntegratorEuler::svIntegratorEuler(DynamicObject* dyn)
    : svIntegratorRungeKutta(dyn, svIntegratorEuler::getCoefficients()) {}

RKCoefficients<1> svIntegratorEuler::getCoefficients() {
    RKCoefficients<1> coefficients;

    coefficients.bArray = {1.};

    return coefficients;
}
