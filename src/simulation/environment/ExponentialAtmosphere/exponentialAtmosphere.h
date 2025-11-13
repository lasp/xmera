#ifndef EXPONENTIAL_ATMOSPHERE_H
#define EXPONENTIAL_ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief exponential atmosphere model */
class ExponentialAtmosphere : public AtmosphereBase {
   public:
    ExponentialAtmosphere();
    ~ExponentialAtmosphere();

   private:
    void evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime);

   public:
    double baseDensity;        //!< [kg/m^3] Density at h=0
    double scaleHeight;        //!< [m] Exponential characteristic height
    double localTemp = 293.0;  //!< [K] Local atmospheric temperature; set to be constant.
    BSKLogger bskLogger;       //!< -- BSK Logging
};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
