/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "thrForceMapping.h"

/*! Reset method for the BSK module adapter interface. This method also calls the algorithm reset method.
 @return void
 @param callTime [ns] Time the method is called
 */
void ThrForceMapping::reset(uint64_t callTime) {
    assert(this->thrConfigInMsg.isLinked() && this->vehConfigInMsg.isLinked() && this->cmdTorqueInMsg.isLinked());

    auto localThrConfigInMsg = THRArrayConfigMsgPayload();
    if (this->thrConfigInMsg.isWritten()) {
        localThrConfigInMsg = this->thrConfigInMsg();
    }

    // Call the algorithm reset method
    this->algorithm.reset(callTime, localThrConfigInMsg);
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
 */
void ThrForceMapping::updateState(uint64_t callTime) {
    auto LrInputMsg = CmdTorqueBodyMsgPayload();
    if (this->cmdTorqueInMsg.isWritten()) {
        LrInputMsg = this->cmdTorqueInMsg();
    }
    auto localVehConfigInMsg = VehicleConfigMsgPayload();
    if (this->vehConfigInMsg.isWritten()) {
        localVehConfigInMsg = this->vehConfigInMsg();
    }

    // Call the algorithm update method
    THRArrayCmdForceMsgPayload thrusterForceOut = this->algorithm.update(callTime, LrInputMsg, localVehConfigInMsg);

    this->thrForceCmdOutMsg.write(&thrusterForceOut, this->moduleID, callTime);
}

/**
 * @brief Get the control axes in the body frame.
 * @return 3x3 matrix representing the control axes in the body frame.
 */
Eigen::Matrix3d ThrForceMapping::getControlAxesB() const { return this->algorithm.getControlAxesB(); }

/**
 * @brief Set the control axes in body frame.
 * @param axes A 3x3 matrix representing the control axes in body frame.
 */
void ThrForceMapping::setControlAxesB(const Eigen::Matrix3d& axes) { this->algorithm.setControlAxesB(axes); }

/**
 * @brief Get the thruster force magnitudes.
 * @return A vector of thruster force magnitudes.
 */
Vector36d ThrForceMapping::getThrForceMag() const { return this->algorithm.getThrForceMag(); }

/**
 * @brief Set the thruster force magnitudes.
 * @param forceMag A vector of thruster force magnitudes.
 */
void ThrForceMapping::setThrForceMag(const Vector36d& forceMag) { this->algorithm.setThrForceMag(forceMag); }

/**
 * @brief Get the sign of the thruster forces.
 * @return The sign of the thruster forces (POSITIVE or NEGATIVE).
 */
ThrForceSign ThrForceMapping::getThrForceSign() const { return this->algorithm.getThrForceSign(); }

/**
 * @brief Set the sign of the thruster forces.
 * @param sign The sign of the thruster forces (POSITIVE or NEGATIVE).
 */
void ThrForceMapping::setThrForceSign(ThrForceSign sign) { this->algorithm.setThrForceSign(sign); }

/**
 * @brief Get the angular error threshold.
 * @return The angular error threshold.
 */
double ThrForceMapping::getAngErrThresh() const { return this->algorithm.getAngErrThresh(); }

/**
 * @brief Set the angular error threshold.
 * @param The new angular error threshold.
 */
void ThrForceMapping::setAngErrThresh(double thresh) { this->algorithm.setAngErrThresh(thresh); }

/**
 * @brief Get the epsilon value.
 * @return The epsilon value.
 */
double ThrForceMapping::getEpsilon() const { return this->algorithm.getEpsilon(); }

/**
 * @brief Set the epsilon value.
 * @param The new epsilon value.
 */
void ThrForceMapping::setEpsilon(double eps) { this->algorithm.setEpsilon(eps); }

/**
 * @brief Check if the second least squares fitting loop should be used.
 * @return True if the 2nd loop should be used, false otherwise.
 */
bool ThrForceMapping::getUse2ndLoop() const { return this->algorithm.getUse2ndLoop(); }

/**
 * @brief Set if the second least squares fitting loop should be used.
 * @return True if the 2nd loop should be used, false otherwise.
 */
void ThrForceMapping::setUse2ndLoop(bool loopFlag) { this->algorithm.setUse2ndLoop(loopFlag); }
