// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef stateVecIntegrator_h
#define stateVecIntegrator_h

class DynamicObject;

/*! @brief state vector integrator class */
struct StateVecIntegrator {
    virtual ~StateVecIntegrator() = default;

    virtual void integrate(DynamicObject &dyn, double currentTime, double timeStep) = 0;
};

#endif /* StateVecIntegrator_h */
