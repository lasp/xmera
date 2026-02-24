// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRK2_h
#define svIntegratorRK2_h

#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h"

/*! @brief 2nd order Runge-Kutta integrator */
class svIntegratorRK2 : public svIntegratorRungeKutta<2> {
   public:
    svIntegratorRK2(DynamicObject* dyn);  //!< class method
   private:
    static RKCoefficients<2> getCoefficients();
};

#endif /* svIntegratorRK2_h */
