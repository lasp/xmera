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
