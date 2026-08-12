// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "dynamicEffector.h"

/*! This is the constructor, just setting the variables to zero */
DynamicEffector::DynamicEffector() {
    // Set forces and torques to zero
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();

    return;
}

/*! This is the destructor, nothing to report here */
DynamicEffector::~DynamicEffector() {
    return;
}
