#ifndef BORE_ANG_CALC_H
#define BORE_ANG_CALC_H

#include <Eigen/Dense>
#include <vector>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/BoreAngleMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>

/*! @brief A class to perform a range of boresight related calculations.
 */
class BoreAngCalc : public SysModel {
   public:
    BoreAngCalc();
    ~BoreAngCalc();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void computeCelestialAxisPoint();
    void computeCelestialOutputData();
    void computeInertialOutputData();
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;          //!< (-) spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> celBodyInMsg;  //!< (-) celestial body state msg at which we pointing at
    Message<BoreAngleMsgPayload> angOutMsg;                //!< (-) bore sight output message

    Eigen::Vector3d boreVec_B;             //!< (-) boresight vector in structure
    Eigen::Vector3d boreVec_Po;            //!< (-) pointing vector in the target relative point frame
    Eigen::Vector3d inertialHeadingVec_N;  //!< (-) inertial boresight vector

   private:
    SpicePlanetStateMsgPayload localPlanet;  //!< (-) planet that we are pointing at
    SCStatesMsgPayload localState;           //!< (-) observed state of the spacecraft

    BoreAngleMsgPayload boresightAng = {};  //!< (-) Boresight angles relative to target
    bool inputsGood = false;                //!< (-) Flag indicating that inputs were read correctly
    bool useCelestialHeading = false;  //!< (-) Flag indicating that the module should use the celestial body heading
    bool useInertialHeading = false;   //!< (-) Flag indicating that the module should use the inertial heading
    BSKLogger bskLogger;               //!< -- BSK Logging
};

#endif
