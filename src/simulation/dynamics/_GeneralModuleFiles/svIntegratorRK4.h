// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorRK4_h
#define svIntegratorRK4_h

#include <simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h>

/*! @brief 4th order Runge-Kutta integrator */
class svIntegratorRK4 : public svIntegratorRungeKutta<4> {
   public:
    svIntegratorRK4(DynamicObject* dyn);  //!< class method
   private:
    static RKCoefficients<4> getCoefficients();
};

#endif /* svIntegratorRK4_h */
