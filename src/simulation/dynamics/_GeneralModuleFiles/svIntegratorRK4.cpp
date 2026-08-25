// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "svIntegratorRK4.h"

svIntegratorRK4::svIntegratorRK4(DynamicObject* dynIn)
    : StateVecIntegrator(dynIn) {}

void svIntegratorRK4::integrate(double currentTime, double timeStep) {
    auto prevState = ExtendedStateVector::fromStates(this->dynPtrs);
    auto nextState = prevState;

    {
        for (auto dynPtr : this->dynPtrs) { dynPtr->equationsOfMotion(currentTime, timeStep); }

        auto kValue = ExtendedStateVector::fromStateDerivs(this->dynPtrs);
        nextState += kValue * (timeStep / 6.0);

        auto predictedState = prevState;
        predictedState += kValue * (timeStep / 2.0);
        predictedState.setStates(this->dynPtrs);
    }

    {
        for (auto dynPtr : this->dynPtrs) { dynPtr->equationsOfMotion(currentTime + timeStep / 2.0, timeStep); }

        auto kValue = ExtendedStateVector::fromStateDerivs(this->dynPtrs);
        nextState += kValue * (timeStep / 3.0);

        auto predictedState = prevState;
        predictedState += kValue * (timeStep / 2.0);
        predictedState.setStates(this->dynPtrs);
    }

    {
        for (auto dynPtr : this->dynPtrs) { dynPtr->equationsOfMotion(currentTime + timeStep / 2.0, timeStep); }

        auto kValue = ExtendedStateVector::fromStateDerivs(this->dynPtrs);
        nextState += kValue * (timeStep / 3.0);

        auto predictedState = prevState;
        predictedState += kValue * timeStep;
        predictedState.setStates(this->dynPtrs);
    }

    {
        for (auto dynPtr : this->dynPtrs) { dynPtr->equationsOfMotion(currentTime + timeStep, timeStep); }

        auto kValue = ExtendedStateVector::fromStateDerivs(this->dynPtrs);
        nextState += kValue * (timeStep / 6.0);
    }

    nextState.setStates(this->dynPtrs);
}
