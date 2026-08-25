// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef stateVecIntegrator_h
#define stateVecIntegrator_h

class DynamicObject;

/*! @brief state vector integrator class */
class StateVecIntegrator {
public:
    StateVecIntegrator(DynamicObject* dynIn) : dynPtr(dynIn) {}

    virtual ~StateVecIntegrator() = default;

    virtual void integrate(double currentTime, double timeStep) = 0;

    //! This is an object that contains the method equationsOfMotion(), also known as the F function.
    DynamicObject* dynPtr;
};

#endif /* StateVecIntegrator_h */
