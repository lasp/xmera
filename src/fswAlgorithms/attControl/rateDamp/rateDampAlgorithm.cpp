#include "rateDampAlgorithm.h"
#include <cassert>
#include <cmath>

/*! Update method for the rateDamp control algorithm. This method computes the required control torque command.
 @return void
 @param currentSimNanos [ns] Time the method is called
 @param attNavInMsg Attitude navigation message
 */
CmdTorqueBodyMsgPayload RateDampAlgorithm::update(uint64_t currentSimNanos, NavAttMsgPayload& attNavInMsg) {
    // Create and populate the output command torque message
    auto cmdTorqueOutBuffer = CmdTorqueBodyMsgPayload();
    for (int i = 0; i < 3; ++i) {
        cmdTorqueOutBuffer.torqueRequestBody[i] = -this->P * attNavInMsg.omega_BN_B[i];
    }

    return cmdTorqueOutBuffer;
}

/*! Set the module rate feedback gain
    @param double P
    @return void
    */
void RateDampAlgorithm::setRateGain(double p) {
    assert(p > 0.0);
    this->P = std::abs(p);
}

/*! Get the module rate feedback gain
    @return double
    */
double RateDampAlgorithm::getRateGain() const { return this->P; }
