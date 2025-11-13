#ifndef XMERA_RATE_DAMP_H
#define XMERA_RATE_DAMP_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include "rateDampAlgorithm.h"

/*! @brief Rate damp class */
class RateDamp : public SysModel {
   public:
    RateDamp() = default;                                 //!< Constructor
    ~RateDamp() = default;                                //!< Destructor
    void reset(uint64_t currentSimNanos) override;        //!< Reset method
    void updateState(uint64_t currentSimNanos) override;  //!< Update method
    void setRateGain(double p);                           //!< Setter method for rate feedback gain
    double getRateGain() const;                           //!< Getter method for rate feedback gain

    ReadFunctor<NavAttMsgPayload> attNavInMsg;         //!< Navigation input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;  //!< Command torque output message

   private:
    RateDampAlgorithm algorithm{};  //!< Algorithm for rateDamp control logic (BSK-agnostic)
};

#endif
