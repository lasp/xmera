// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRK4_h
#define svIntegratorRK4_h

#include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>

//! 4th order Runge-Kutta integrator
struct svIntegratorRK4 final : public StateVecIntegrator {
    //! Performs the integration of the associated dynamic object up to time currentTime+timeStep
    void integrate(DynamicObject &dyn, double currentTime, double timeStep) override {
        auto prevState = dyn.dynManager.stateContainer;
        auto nextState = prevState;

        dyn.equationsOfMotion(currentTime, timeStep);
        nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        nextState.propagateState(timeStep / 6.0);

        dyn.dynManager.updateStateVector(prevState);
        dyn.dynManager.propagateStateVector(timeStep / 2.0);
        dyn.equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        nextState.propagateState(timeStep / 3.0);

        dyn.dynManager.updateStateVector(prevState);
        dyn.dynManager.propagateStateVector(timeStep / 2.0);
        dyn.equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        nextState.setDerivativesFrom(dyn.dynManager.stateContainer);
        nextState.propagateState(timeStep / 3.0);

        dyn.dynManager.updateStateVector(prevState);
        dyn.dynManager.propagateStateVector(timeStep);
        dyn.equationsOfMotion(currentTime + timeStep, timeStep);

        dyn.dynManager.updateStateVector(nextState);
        dyn.dynManager.propagateStateVector(timeStep / 6.0);
    }
};

#endif
