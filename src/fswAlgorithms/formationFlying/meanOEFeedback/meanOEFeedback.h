#ifndef _MEAN_OE_FEEDBACK_H_
#define _MEAN_OE_FEEDBACK_H_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the sub-module routines. */
class MeanOEFeedback : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void calcLyapunovFeedback(NavTransMsgPayload chiefTransMsg,
                              NavTransMsgPayload deputyTransMsg,
                              CmdForceInertialMsgPayload* forceMsg);
    ReadFunctor<NavTransMsgPayload> chiefTransInMsg;   //!< chief orbit input message
    ReadFunctor<NavTransMsgPayload> deputyTransInMsg;  //!< deputy orbit input message
    Message<CmdForceInertialMsgPayload> forceOutMsg;   //!< deputy control force output message

    double K[36];                //!< Lyapunov Gain (6*6)
    double targetDiffOeMean[6];  //!< target mean orbital element difference
    uint8_t oeType;              //!< 0: classic (default), 1: equinoctial
    double mu;                   //!< [m^3/s^2] gravitational constant
    double req;                  //!< [m] equatorial planet radius
    double J2;                   //!< [] J2 planet oblateness parameter
    BSKLogger bskLogger = {};    //!< BSK Logging
};

#endif
