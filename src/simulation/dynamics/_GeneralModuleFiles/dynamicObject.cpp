// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "dynamicObject.h"

#include <architecture/utilities/logging.h>

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator) {
    if (!newIntegrator) {
        xmera::logging::get(this->modelTag)->error("New integrator cannot be a null pointer");
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
