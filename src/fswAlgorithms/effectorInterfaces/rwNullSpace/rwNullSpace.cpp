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

#include "rwNullSpace.h"

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
    RWConstellationMsgPayload localRWData = this->rwConfigInMsg(); /* local copy of RW configuration data */

    /* create the 3xN [Gs] RW spin axis projection matrix */
    uint32_t numWheels = (uint32_t)localRWData.numRW;
    if (numWheels > RW_EFF_CNT) {
        throw std::invalid_argument("rwNullSpace.numWheels is larger than max effector count.");
    }

    this->algorithm.reset(localRWData);
}

/*! This method takes the input reaction wheel commands as well as the observed
    reaction wheel speeds and balances the commands so that the overall vehicle
    momentum is minimized.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwNullSpace::updateState(uint64_t callTime) {
    RwMotorTorqueMsgPayload controlRequest =
        this->rwMotorTorqueInMsg();                     /* [Nm]  array of the RW motor torque solution */
    RWSpeedMsgPayload rwSpeeds = this->rwSpeedsInMsg(); /* [r/s] array of RW speeds */
    RWSpeedMsgPayload rwDesiredSpeeds{};                /* [r/s] array of RW speeds */

    if (this->rwDesiredSpeedsInMsg.isLinked()) {
        rwDesiredSpeeds = this->rwDesiredSpeedsInMsg();
    }

    RwMotorTorqueMsgPayload finalControl = this->algorithm.update(controlRequest, rwSpeeds, rwDesiredSpeeds);

    /* write the final RW torque solution to the output message */
    this->rwMotorTorqueOutMsg.write(&finalControl, this->moduleID, callTime);
}

/**
 * @brief Set the gain used for the wheel speed difference.
 * @param gain The gain used for the wheel speed difference.
 */
void RwNullSpace::setOmegaGain(const double gain) { this->algorithm.setOmegaGain(gain); }

/**
 * @brief Get the gain used for the wheel speed difference.
 * @return double The gain used for the wheel speed difference.
 */
double RwNullSpace::getOmegaGain() const { return this->algorithm.getOmegaGain(); }
