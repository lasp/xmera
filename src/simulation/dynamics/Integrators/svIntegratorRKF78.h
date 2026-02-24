// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRKF78_h
#define svIntegratorRKF78_h

#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"

/*! @brief 7/8 order Runge-Kutta integrator */
class svIntegratorRKF78 : public svIntegratorAdaptiveRungeKutta<13> {
   public:
    svIntegratorRKF78(DynamicObject* dyn);  //!< class method
   private:
    static RKAdaptiveCoefficients<13> getCoefficients();
};

#endif /* svIntegratorRKF78_h */
