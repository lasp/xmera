/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "rateServoFullNonlinear.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RateServoFullNonlinear::reset(uint64_t callTime) {
    /* make sure option msg connections are correctly done */
    if (this->rwParamsInMsg.isLinked()) {
        if (!this->rwSpeedsInMsg.isLinked()) {
            throw std::invalid_argument(
                "rateServoFullNonlinear.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
        }
    }

    // check if essential messages are connected
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.guidInMsg wasn't connected.");
    }

    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.vehConfigInMsg wasn't connected.");
    }

    if (!this->rateSteeringInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.rateSteeringInMsg wasn't connected.");
    }

    VehicleConfigMsgPayload sc = this->vehConfigInMsg();
    RWArrayConfigMsgPayload rwConfigParams{};
    bool rwParamsIsLinked{};

    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
        rwParamsIsLinked = true;
    }
    this->numRW = rwConfigParams.numRW;

    this->algorithm.reset(sc, rwConfigParams, rwParamsIsLinked);
}

/*! This method takes and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RateServoFullNonlinear::updateState(uint64_t callTime) {
    AttGuidMsgPayload guidCmd{};                   /*!< Guidance input Message */
    RWSpeedMsgPayload wheelSpeeds{};               /*!< Reaction wheel speed estimates input message */
    RWAvailabilityMsgPayload wheelsAvailability{}; /*!< Reaction wheel availability input message */
    RateCmdMsgPayload rateGuid{};                  /*!< rate steering law message input message */

    guidCmd = this->guidInMsg();
    rateGuid = this->rateSteeringInMsg();

    if (this->numRW > 0) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    CmdTorqueBodyMsgPayload controlOut = algorithm.update(callTime, guidCmd, rateGuid, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);
}

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void RateServoFullNonlinear::setP(const double gain) { this->algorithm.setP(gain); }

/*! Getter method for the gain P.
 @return const double
*/
double RateServoFullNonlinear::getP() const { return this->algorithm.getP(); }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void RateServoFullNonlinear::setKi(const double gain) { this->algorithm.setKi(gain); }

/*! Getter method for the gain Ki.
 @return const double
*/
double RateServoFullNonlinear::getKi() const { return this->algorithm.getKi(); }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void RateServoFullNonlinear::setIntegralLimit(const double limit) { this->algorithm.setIntegralLimit(limit); }

/*! Getter method for the integral limit.
 @return const double
*/
double RateServoFullNonlinear::getIntegralLimit() const { return this->algorithm.getIntegralLimit(); }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateServoFullNonlinear::setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B) {
    this->algorithm.setKnownTorquePntB_B(knownTorquePntB_B);
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
Eigen::Vector3d RateServoFullNonlinear::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }
