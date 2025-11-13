#ifndef LOCATIONPOINTING_H
#define LOCATIONPOINTING_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/GroundStateMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief This module is used to generate the attitude reference message in order to have a spacecraft point at a
 * location on the ground
 */
class LocationPointing : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* user configurable variables */
    double pHat_B[3];            /*!< body fixed vector that is to be aimed at a location */
    double smallAngle;           /*!< rad An angle value that specifies what is near 0 or 180 degrees */
    int useBoresightRateDamping; /*!< [int] flag to use rate damping about the sensor boresight */

    /* private variables */
    double sigma_BR_old[3]; /*!< Older sigma_BR value, stored for finite diff*/
    uint64_t time_old;      /*!< [ns] prior time value */
    double init;            /*!< moudle initialization counter */
    double eHat180_B[3];    /*!< -- Eigen axis to use if commanded axis is 180 from pHat */

    /* declare module IO interfaces */
    ReadFunctor<NavAttMsgPayload> scAttInMsg;          //!< input msg with inertial spacecraft attitude states
    ReadFunctor<NavTransMsgPayload> scTransInMsg;      //!< input msg with inertial spacecraft position states
    ReadFunctor<GroundStateMsgPayload> locationInMsg;  //!< input msg with location relative to planet
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;     //!< input celestial body message
    ReadFunctor<NavTransMsgPayload> scTargetInMsg;     //!< input msg with inertial target spacecraft position states
    Message<AttGuidMsgPayload> attGuidOutMsg;          //!< attitude guidance output message
    Message<AttRefMsgPayload> attRefOutMsg;            //!< attitude reference output message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
