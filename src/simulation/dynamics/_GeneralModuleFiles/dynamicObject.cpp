// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "dynamicObject.h"

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator) {
    if (!newIntegrator) {
        bskLogger.bskLog(BSK_ERROR, "New integrator cannot be a null pointer");
        return;
    }

    if (newIntegrator->dynPtr != this) {
        bskLogger.bskLog(BSK_ERROR, "New integrator must have been created using this DynamicObject");
        return;
    }

    // If there was already an integrator set, then whatever dynPtr that the
    // original integrator had take priority over the dynPtr of newIntegrator
    if (this->integrator) {
        newIntegrator->dynPtr = this->integrator->dynPtr;
    }

    delete this->integrator;

    this->integrator = newIntegrator;
}

void DynamicObject::integrateState(double integrateToThisTime) {
    this->integrator->dynPtr->preIntegration(integrateToThisTime);
    this->integrator->integrate(this->timeBefore, this->timeStep);
    this->integrator->dynPtr->postIntegration(integrateToThisTime);
}
