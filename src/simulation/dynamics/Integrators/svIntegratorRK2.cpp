// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "svIntegratorRK2.h"

svIntegratorRK2::svIntegratorRK2(DynamicObject* dyn)
    : svIntegratorRungeKutta(dyn, svIntegratorRK2::getCoefficients()) {}

RKCoefficients<2> svIntegratorRK2::getCoefficients() {
    RKCoefficients<2> coefficients;
    coefficients.aMatrix.at(1).at(0) = 1;
    coefficients.aMatrix.at(1).at(1) = 1;

    coefficients.bArray = {0.5, 0.5};

    coefficients.cArray = {0., 1.};

    return coefficients;
}
