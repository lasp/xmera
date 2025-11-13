#include "thrFiringSchmittAlgorithm.h"

#include <architecture/utilities/macroDefinitions.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmittAlgorithm::reset(uint64_t callTime, THRArrayConfigMsgPayload const& thrusterConfigPayload) {
    this->prevCallTime = 0;

    /*! - store the number of installed thrusters */
    this->numThrusters = thrusterConfigPayload.numThrusters;
    this->lastThrustState.fill(false);
    /*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
    for (uint32_t i = 0; i < this->numThrusters; i++) {
        this->maxThrust[i] = thrusterConfigPayload.thrusters[i].maxThrust;
    }
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
THRArrayOnTimeCmdMsgPayload ThrFiringSchmittAlgorithm::update(uint64_t callTime,
                                                              THRArrayCmdForceMsgPayload& thrForceIn) {
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut{}; /* -- thruster on-time output payload */

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
    if (this->prevCallTime == 0) {
        this->prevCallTime = callTime;

        for (uint32_t i = 0; i < this->numThrusters; i++) {
            thrOnTimeOut.OnTimeRequest[i] = (double)(this->baseThrustState) * 2.0;
        }

        return thrOnTimeOut;
    }

    /*! - compute control time period Delta_t */
    double controlPeriod = ((double)(callTime - this->prevCallTime)) * NANO2SEC; /* [s] control period */
    this->prevCallTime = callTime;

    std::array<double, MAX_EFF_CNT> onTime{}; /* [s] array of commanded on time for thrusters */
                                              /*! - Loop through thrusters */
    for (uint32_t i = 0; i < this->numThrusters; i++) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (this->baseThrustState == PulsingRegime::OFFPULSING) {
            thrForceIn.thrForce[i] += this->maxThrust[i];
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrForceIn.thrForce[i] < 0.0) {
            thrForceIn.thrForce[i] = 0.0;
        }
        /*! - Compute T_on from thrust request, max thrust, and control period */
        onTime[i] = thrForceIn.thrForce[i] / this->maxThrust[i] * controlPeriod;

        /*! - Apply Schmitt trigger logic */
        if (onTime[i] < this->thrMinFireTime) {
            /*! - Request is less than minimum fire time */
            double level = onTime[i] / this->thrMinFireTime; /* [-] duty cycle fraction */
            if (level >= this->levelOn) {
                this->lastThrustState[i] = true;
                onTime[i] = this->thrMinFireTime;
            } else if (level <= this->levelOff) {
                this->lastThrustState[i] = false;
                onTime[i] = 0.0;
            } else if (this->lastThrustState[i]) {
                onTime[i] = this->thrMinFireTime;
            } else {
                onTime[i] = 0.0;
            }
        } else if (onTime[i] >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
            this->lastThrustState[i] = true;
            onTime[i] = 1.1 * controlPeriod;  // oversaturate to avoid numerical error
        } else {
            /*! - Request is greater than minimum fire time and less than control period */
            this->lastThrustState[i] = true;
        }

        /*! Set the output data */
        thrOnTimeOut.OnTimeRequest[i] = onTime[i];
    }

    return thrOnTimeOut;
}

/**
 * @brief Get the ON duty cycle fraction.
 * @return double The current ON duty cycle fraction.
 */
double ThrFiringSchmittAlgorithm::getLevelOn() const { return this->levelOn; }

/**
 * @brief Set the ON duty cycle fraction.
 * @param level The new ON duty cycle fraction to set.
 */
void ThrFiringSchmittAlgorithm::setLevelOn(double level) { this->levelOn = level; }

/**
 * @brief Get the OFF duty cycle fraction.
 * @return double The current OFF duty cycle fraction.
 */
double ThrFiringSchmittAlgorithm::getLevelOff() const { return this->levelOff; }

/**
 * @brief Set the OFF duty cycle fraction.
 * @param level The new OFF duty cycle fraction to set.
 */
void ThrFiringSchmittAlgorithm::setLevelOff(double level) { this->levelOff = level; }

/**
 * @brief Get the minimum ON time for thrusters.
 * @return double The current minimum ON time in seconds.
 */
double ThrFiringSchmittAlgorithm::getThrMinFireTime() const { return this->thrMinFireTime; }

/**
 * @brief Set the minimum ON time for thrusters.
 * @param time The new minimum ON time in seconds to set.
 */
void ThrFiringSchmittAlgorithm::setThrMinFireTime(double time) { this->thrMinFireTime = time; }

/**
 * @brief Get the base thrust state.
 * @return int The current base thrust state (0 for off-pulsing, 1 for on-pulsing).
 */
PulsingRegime ThrFiringSchmittAlgorithm::getPulsingRegime() const { return this->baseThrustState; }

/**
 * @brief Set the base thrust state.
 * @param state The new base thrust state to set (0 for off-pulsing, 1 for on-pulsing).
 */
void ThrFiringSchmittAlgorithm::setPulsingRegime(PulsingRegime state) { this->baseThrustState = state; }
