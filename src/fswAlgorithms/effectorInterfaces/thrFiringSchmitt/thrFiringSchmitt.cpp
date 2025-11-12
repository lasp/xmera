#include "thrFiringSchmitt.h"

ThrFiringSchmitt::ThrFiringSchmitt() { this->algorithm = ThrFiringSchmittAlgorithm(); }

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->thrConfInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringSchmitt.thrConfInMsg wasn't connected.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringSchmitt.thrForceInMsg wasn't connected.");
    }

    this->algorithm.reset(callTime, this->thrConfInMsg());
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmitt::updateState(uint64_t callTime) {
    THRArrayCmdForceMsgPayload thrForceIn = this->thrForceInMsg();
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = this->algorithm.update(callTime, thrForceIn);
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/**
 * @brief Get the ON duty cycle fraction.
 * @return double The current ON duty cycle fraction.
 */
double ThrFiringSchmitt::getLevelOn() const { return this->algorithm.getLevelOn(); }

/**
 * @brief Set the ON duty cycle fraction.
 * @param level The new ON duty cycle fraction to set.
 */
void ThrFiringSchmitt::setLevelOn(double level) { this->algorithm.setLevelOn(level); }

/**
 * @brief Get the OFF duty cycle fraction.
 * @return double The current OFF duty cycle fraction.
 */
double ThrFiringSchmitt::getLevelOff() const { return this->algorithm.getLevelOff(); }

/**
 * @brief Set the OFF duty cycle fraction.
 * @param level The new OFF duty cycle fraction to set.
 */
void ThrFiringSchmitt::setLevelOff(double level) { this->algorithm.setLevelOff(level); }

/**
 * @brief Get the minimum ON time for thrusters.
 * @return double The current minimum ON time in seconds.
 */
double ThrFiringSchmitt::getThrMinFireTime() const { return this->algorithm.getThrMinFireTime(); }

/**
 * @brief Set the minimum ON time for thrusters.
 * @param time The new minimum ON time in seconds to set.
 */
void ThrFiringSchmitt::setThrMinFireTime(double time) { this->algorithm.setThrMinFireTime(time); }

/**
 * @brief Get the base thrust state.
 * @return int The current base thrust state (0 for off-pulsing, 1 for on-pulsing).
 */
uint32_t ThrFiringSchmitt::getBaseThrustState() const {
    return static_cast<uint32_t>(this->algorithm.getPulsingRegime());
}

/**
 * @brief Set the base thrust state.
 * @param state The new base thrust state to set (0 for off-pulsing, 1 for on-pulsing).
 */
void ThrFiringSchmitt::setBaseThrustState(uint32_t state) {
    this->algorithm.setPulsingRegime(static_cast<PulsingRegime>(state));
}
