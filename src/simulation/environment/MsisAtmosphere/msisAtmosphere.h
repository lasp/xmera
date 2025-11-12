#ifndef MSIS_ATMOSPHERE_H
#define MSIS_ATMOSPHERE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <time.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>

#include <architecture/msgPayloadDef/SwDataMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

extern "C" {
#include "nrlmsise-00.h"
}

/*! @brief MSIS athomsphere model */
class MsisAtmosphere : public AtmosphereBase {
   public:
    MsisAtmosphere();
    ~MsisAtmosphere();

   private:
    void customWriteMessages(uint64_t CurrentClock);
    bool customReadMessages();
    void customreset(uint64_t CurrentClock);
    bool ReadInputs();
    void updateInputParams();
    void updateSwIndices();
    void evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime);
    void customSetEpochFromVariable();

   public:
    std::vector<ReadFunctor<SwDataMsgPayload>> swDataInMsgs;  //!< Vector of space weather input message names
    int epochDoy;                                             //!< [day] Day-of-Year at epoch
    BSKLogger bskLogger;                                      //!< -- BSK Logging

   private:
    Eigen::Vector3d currentLLA;  //!< [-] Container for local Latitude, Longitude, Altitude geodetic position; units are
                                 //!< rad and km respectively.
    std::vector<SwDataMsgPayload> swDataList;  //!< Vector of space weather messages

    // NRLMSISE-00 Specific attributes
    nrlmsise_input msisInput;    //!< Struct containing NRLMSISE-00 input values; see their doc for details.
    nrlmsise_output msisOutput;  //!< Struct containing NRLMSISE-00 output values; see their doc for details.
    nrlmsise_flags msisFlags;
    ap_array aph;
    double ap;
    double f107;
    double f107A;
};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
