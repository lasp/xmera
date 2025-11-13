#ifndef _HILL_TO_ATT_H
#define _HILL_TO_ATT_H

#include <stdint.h>
#include <string.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/HillRelStateMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the sub-module routines. */
class HillToAttRef : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    AttRefMsgPayload relativeToInertialMRP(double relativeAtt[3], double sigma_XN[3]);

    /* declare module IO interfaces */
    ReadFunctor<HillRelStateMsgPayload> hillStateInMsg;  //!< Provides state relative to chief
    ReadFunctor<AttRefMsgPayload> attRefInMsg;           //!< (Optional) Provides basis for relative attitude
    ReadFunctor<NavAttMsgPayload> attNavInMsg;           //!< (Optional) Provides basis for relative attitude
    Message<AttRefMsgPayload> attRefOutMsg;              //!< Provides the attitude reference output message.
    BSKLogger bskLogger = {};                            //!< BSK Logging

    double gainMatrix[3][6];  //!< User-configured gain matrix that maps from hill states to relative attitudes.
    double relMRPMin;         //!< Minimum value for the relative MRP components; user-configurable.
    double relMRPMax;         //!< Maximum value for the relative MRP components; user-configurable.
};

#endif
