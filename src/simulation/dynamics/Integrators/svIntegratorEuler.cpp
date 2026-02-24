// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "svIntegratorEuler.h"

svIntegratorEuler::svIntegratorEuler(DynamicObject* dyn)
    : svIntegratorRungeKutta(dyn, svIntegratorEuler::getCoefficients()) {}

RKCoefficients<1> svIntegratorEuler::getCoefficients() {
    RKCoefficients<1> coefficients;

    coefficients.bArray = {1.};

    return coefficients;
}
