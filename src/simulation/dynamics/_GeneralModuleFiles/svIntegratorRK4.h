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
