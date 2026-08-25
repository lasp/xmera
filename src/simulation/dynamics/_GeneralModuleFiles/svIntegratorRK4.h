// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRK4_h
#define svIntegratorRK4_h

#include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>
#include <simulation/dynamics/_GeneralModuleFiles/extendedStateVector.h>

/*! @brief 4th order Runge-Kutta integrator */
class svIntegratorRK4 final : public StateVecIntegrator {
public:
    svIntegratorRK4(DynamicObject* dyn);  //!< class method

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep
     */
    void integrate(double currentTime, double timeStep) override;
};

#endif /* svIntegratorRK4_h */
