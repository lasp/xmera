// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "svIntegratorRK4.h"

svIntegratorRK4::svIntegratorRK4(DynamicObject* dyn)
    : svIntegratorRungeKutta(dyn, svIntegratorRK4::getCoefficients()) {}

RKCoefficients<4> svIntegratorRK4::getCoefficients() {
    RKCoefficients<4> coefficients;
    coefficients.aMatrix.at(1).at(0) = 0.5;
    coefficients.aMatrix.at(2).at(1) = 0.5;
    coefficients.aMatrix.at(3).at(2) = 1.0;

    coefficients.bArray = {1. / 6., 1. / 3., 1. / 3., 1. / 6.};

    coefficients.cArray = {0., 1. / 2., 1. / 2., 1.};

    return coefficients;
}
