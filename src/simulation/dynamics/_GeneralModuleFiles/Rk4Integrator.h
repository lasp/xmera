// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRK4_h
#define svIntegratorRK4_h

#include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>

//! 4th order Runge-Kutta integrator
struct Rk4Integrator final : public StateVecIntegrator {
    //! Performs the integration of the associated dynamic object up to time currentTime+timeStep
    void integrate(DynamicObject &dyn, double currentTime, double timeStep) override {
        this->prevState = dyn.dynManager.stateContainer;
        this->nextState = this->prevState;

        dyn.equationsOfMotion(currentTime, timeStep);
        this->nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        this->nextState.propagateState(timeStep / 6.0);

        dyn.dynManager.updateStateVector(this->prevState);
        dyn.dynManager.propagateStateVector(timeStep / 2.0);
        dyn.equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        this->nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        this->nextState.propagateState(timeStep / 3.0);

        dyn.dynManager.updateStateVector(this->prevState);
        dyn.dynManager.propagateStateVector(timeStep / 2.0);
        dyn.equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        this->nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        this->nextState.propagateState(timeStep / 3.0);

        dyn.dynManager.updateStateVector(this->prevState);
        dyn.dynManager.propagateStateVector(timeStep);
        dyn.equationsOfMotion(currentTime + timeStep, timeStep);

        dyn.dynManager.updateStateVector(this->nextState);
        dyn.dynManager.propagateStateVector(timeStep / 6.0);
    }

private:
    // A cached allocation for reuse, under the assumption that repeated calls to `integrate`
    // use the same `DynamicObject` with the same named states.
    mutable StateVector prevState;
    mutable StateVector nextState;
};

#endif
