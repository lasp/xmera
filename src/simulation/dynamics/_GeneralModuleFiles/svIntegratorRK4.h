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
    svIntegratorRK4(DynamicObject* dyn)
        : StateVecIntegrator(dyn) {}

    //! Performs the integration of the associated dynamic object up to time currentTime+timeStep
    void integrate(double currentTime, double timeStep) override {
        auto prevState = this->dynPtr->dynManager.stateContainer;
        auto nextState = prevState;

        this->dynPtr->equationsOfMotion(currentTime, timeStep);
        nextState.setDerivativesFrom(this->dynPtr->dynManager.stateContainer);
        nextState.propagateState(timeStep / 6.0);

        this->dynPtr->dynManager.updateStateVector(prevState);
        this->dynPtr->dynManager.propagateStateVector(timeStep / 2.0);
        this->dynPtr->equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        nextState.setDerivativesFrom(this->dynPtr->dynManager.stateContainer);
        nextState.propagateState(timeStep / 3.0);

        this->dynPtr->dynManager.updateStateVector(prevState);
        this->dynPtr->dynManager.propagateStateVector(timeStep / 2.0);
        this->dynPtr->equationsOfMotion(currentTime + timeStep / 2.0, timeStep);
        nextState.setDerivativesFrom(this->dynPtr->dynManager.stateContainer);
        nextState.propagateState(timeStep / 3.0);

        this->dynPtr->dynManager.updateStateVector(prevState);
        this->dynPtr->dynManager.propagateStateVector(timeStep);
        this->dynPtr->equationsOfMotion(currentTime + timeStep, timeStep);

        this->dynPtr->dynManager.updateStateVector(nextState);
        this->dynPtr->dynManager.propagateStateVector(timeStep / 6.0);
    }
};

#endif
