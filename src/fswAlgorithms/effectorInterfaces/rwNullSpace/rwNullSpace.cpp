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

#include "fswAlgorithms/effectorInterfaces/rwNullSpace/rwNullSpace.h"
#include "architecture/utilities/eigenSupport.h"

#include <stdexcept>

/*! @brief This resets the module to original states by reading in the RW configuration messages and recreating any
   module specific variables.  The output message is reset to zero.
    @return void
    @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwNullSpace::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwConfigInMsg.isLinked()) {
        throw std::invalid_argument("rwNullSpace.rwConfigInMsg wasn't connected.");
    }
    if (!this->rwMotorTorqueInMsg.isLinked()) {
        throw std::invalid_argument("rwNullSpace.rwMotorTorqueInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("rwNullSpace.rwSpeedsInMsg wasn't connected.");
    }

    /* read in the RW spin axis headings */
    RWConstellationMsgPayload localRWData = this->rwConfigInMsg();  /* local copy of RW configuration data */

    /* create the 3xN [Gs] RW spin axis projection matrix */
    this->numWheels = (uint32_t)localRWData.numRW;
    if (this->numWheels > RW_EFF_CNT) {
        throw std::invalid_argument("rwNullSpace.numWheels is larger than max effector count.");
    }

    Eigen::Matrix<double, 3, RW_EFF_CNT> G_s_B{};
    G_s_B.setZero();
    for(uint32_t i=0; i<this->numWheels; i=i+1)
    {
        G_s_B.col(i) = cArrayAsEigenVector(localRWData.reactionWheels[i].gsHat_B);
    }

    /* find the [tau] null space projection matrix [tau] = ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
    this->tau = Eigen::Matrix<double, RW_EFF_CNT, RW_EFF_CNT>::Identity()
                - G_s_B.transpose() * (G_s_B * G_s_B.transpose()).inverse() * G_s_B;
}

/*! This method takes the input reaction wheel commands as well as the observed
    reaction wheel speeds and balances the commands so that the overall vehicle
    momentum is minimized.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwNullSpace::updateState(uint64_t callTime) {
    RwMotorTorqueMsgPayload controlRequest = this->rwMotorTorqueInMsg(); /* [Nm]  array of the RW motor torque solution */
    RWSpeedMsgPayload rwSpeeds = this->rwSpeedsInMsg(); /* [r/s] array of RW speeds */
    RWSpeedMsgPayload rwDesiredSpeeds{};    /* [r/s] array of RW speeds */
    RwMotorTorqueMsgPayload finalControl{}; /* [Nm]  array of final RW motor torques containing both
                                            the control and null motion torques */

    if (this->rwDesiredSpeedsInMsg.isLinked()) {
        rwDesiredSpeeds = this->rwDesiredSpeedsInMsg();
    }

    /* compute the wheel speed control vector d = -K.DeltaOmega */
    Eigen::Vector<double, MAX_EFF_CNT> d = -this->omegaGain *
        (cArrayAsEigenVector(rwSpeeds.wheelSpeeds) - cArrayAsEigenVector(rwDesiredSpeeds.wheelSpeeds));

    /* compute the RW null space motor torque solution to reduce the wheel speeds */
    Eigen::Vector<double, MAX_EFF_CNT> motorTorqueNullSpace = this->tau * d;

    /* add the null motion RW torque solution to the RW feedback control torque solution */
    Eigen::Vector<double, MAX_EFF_CNT> motorTorque = motorTorqueNullSpace + cArrayAsEigenVector(controlRequest.motorTorque);

    eigenVectorToCArray(motorTorque, finalControl.motorTorque);

    /* write the final RW torque solution to the output message */
    this->rwMotorTorqueOutMsg.write(&finalControl, this->moduleID, callTime);
}

/**
 * @brief Set the gain used for the wheel speed difference.
 * @param gain The gain used for the wheel speed difference.
 */
void RwNullSpace::setOmegaGain(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("Feedback gain must not be negative");
    }
    this->omegaGain = gain;
}

/**
 * @brief Get the gain used for the wheel speed difference.
 * @return double The gain used for the wheel speed difference.
 */
double RwNullSpace::getOmegaGain() const { return this->omegaGain; }
