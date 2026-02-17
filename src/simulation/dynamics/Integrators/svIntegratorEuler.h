// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef svIntegratorEuler_h
#define svIntegratorEuler_h

#include <simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h>

/*! @brief Euler integrator */
class svIntegratorEuler : public svIntegratorRungeKutta<1> {
   public:
    svIntegratorEuler(DynamicObject* dyn);  //!< class method
   private:
    static RKCoefficients<1> getCoefficients();
};

#endif /* svIntegratorEuler_h */
