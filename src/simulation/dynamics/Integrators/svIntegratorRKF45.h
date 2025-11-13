#ifndef svIntegratorRKF45_h
#define svIntegratorRKF45_h

#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"

/*! @brief 4th order Runge-Kutta-Fehlberg variable time step integrator */
class svIntegratorRKF45 : public svIntegratorAdaptiveRungeKutta<6> {
   public:
    svIntegratorRKF45(DynamicObject* dyn);  //!< class method
   private:
    static RKAdaptiveCoefficients<6> getCoefficients();
};

#endif /* svIntegratorRKF45_h */
