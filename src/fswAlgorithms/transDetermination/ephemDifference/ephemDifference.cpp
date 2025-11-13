#include "ephemDifference.h"

/*! @brief This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemDifference::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->ephBaseInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: ephemDifference.ephBaseInMsg wasn't connected.");
    }

    this->ephBdyCount = 0;
    for (const auto& cfg : this->changeBodies) {
        if (cfg.ephInMsg.isLinked()) {
            this->ephBdyCount++;
        } else {
            break;
        }
    }

    if (this->ephBdyCount == 0) {
        this->bskLogger.bskLog(BSK_WARNING,
                               "Your outgoing ephemeris message count is zero. "
                               "Be sure to specify desired output messages.");
    }
}

/*! @brief This method recomputes the body positions and velocities relative to
    the base body ephemeris and writes out updated ephemeris position and velocity
    for each body.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemDifference::updateState(uint64_t callTime) {
    // read input msg
    EphemerisMsgPayload tmpBaseEphem = this->ephBaseInMsg();

    for (uint32_t i = 0; i < this->ephBdyCount; i++) {
        auto tmpEphStore = this->changeBodies[i].ephInMsg();

        for (uint32_t j = 0; j < 3; j++) {
            tmpEphStore.r_BdyZero_N[j] -= tmpBaseEphem.r_BdyZero_N[j];
            tmpEphStore.v_BdyZero_N[j] -= tmpBaseEphem.v_BdyZero_N[j];
        }

        this->changeBodies[i].ephOutMsg.write(&tmpEphStore, moduleID, callTime);
    }
}
