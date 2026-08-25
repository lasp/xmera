// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "dynamicObject.h"

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator) {
    if (!newIntegrator) {
        bskLogger.bskLog(BSK_ERROR, "New integrator cannot be a null pointer");
        return;
    }

    delete this->integrator;
    this->integrator = newIntegrator;
}

void DynamicObject::integrateState(double integrateToThisTime) {
    this->preIntegration(integrateToThisTime);
    this->integrator->integrate(*this, this->timeBefore, this->timeStep);
    this->postIntegration(integrateToThisTime);
}
