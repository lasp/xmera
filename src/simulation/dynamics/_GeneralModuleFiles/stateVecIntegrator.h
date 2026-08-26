// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef stateVecIntegrator_h
#define stateVecIntegrator_h

class DynamicObject;

/*! @brief state vector integrator class */
struct StateVecIntegrator {
    virtual ~StateVecIntegrator() = default;

    /*! Advances the states of the given dynamic object one time step.
        @param dyn dynamic object whose states are advanced
        @param currentTime [s] time at the start of the step
        @param timeStep [s] length of the step
     */
    virtual void integrate(DynamicObject &dyn, double currentTime, double timeStep) = 0;
};

#endif /* StateVecIntegrator_h */
