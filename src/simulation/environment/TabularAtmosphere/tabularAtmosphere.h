#ifndef TABULAR_ATMOSPHERE_H
#define TABULAR_ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief tabular atmosphere model */
class TabularAtmosphere : public AtmosphereBase {
   private:
    // pulls density and temperature from atmospheric table at requested altitude, performs linear interpolation if
    // necessary
    void evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime);

    int altList_length;   // length of list of altitude values extracted from the atmosphere table
    int rhoList_length;   // length of list of density values extracted from the atmosphere table
    int tempList_length;  // length of list of temperature values extracted from the atmosphere table

    virtual void customreset(uint64_t CurrentClock);  // reset if error thrown

   public:
    TabularAtmosphere();
    ~TabularAtmosphere();
    std::vector<double> altList;   //!< vector of doubles of altitude values extracted from the atmosphere table
    std::vector<double> rhoList;   //!< vector of doubles of density values extracted from the atmosphere table
    std::vector<double> tempList;  //!< vector of doubles of temperature values extracted from the atmosphere table
    BSKLogger bskLogger;           //!< -- BSK Logging
};

#endif /* TABULAR_ATMOSPHERE_H */
