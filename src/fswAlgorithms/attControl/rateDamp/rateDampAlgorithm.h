#ifndef _RATE_DAMP_ALGORITHM_H
#define _RATE_DAMP_ALGORITHM_H

#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <cstdint>

/*! @brief Rate damp algorithm class */
class RateDampAlgorithm {
   public:
    CmdTorqueBodyMsgPayload update(uint64_t currentSimNanos,
                                   NavAttMsgPayload& attNavInMsg);  //!< Algorithm update method
    void setRateGain(double p);                                     //!< Setter method for rate feedback gain
    double getRateGain() const;                                     //!< Getter method for rate feedback gain

   private:
    double P{};  //!< [N*m*s] Rate feedback gain
};

#endif
