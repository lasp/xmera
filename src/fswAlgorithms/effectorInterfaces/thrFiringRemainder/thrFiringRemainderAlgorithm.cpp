/*
 ISC License

 Copyright (c) 2016, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#include "thrFiringRemainderAlgorithm.h"
#include "thrFiringRemainderTypes.h"

#include "architecture/utilities/macroDefinitions.h"

#include <array>
#include <stdexcept>

/*! This method performs a complete reset of the algorithm.  Local algorithm variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param thrConfigInMsgPayload The thruster configuration data
 */
void ThrFiringRemainderAlgorithm::reset(const THRArrayConfigMsgPayload& thrConfigInMsgPayload) {
    this->prevCallTime = 0;
    /*! - store the number of installed thrusters */
    this->numThrusters = thrConfigInMsgPayload.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
    for (int i = 0; i < this->numThrusters; i++) {
        this->maxThrust[i] = thrConfigInMsgPayload.thrusters[i].maxThrust;
        this->pulseRemainder[i] = 0.0;
    }

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    this->defaultControlPeriod = 0.0 == this->defaultControlPeriod ? 2.0 : this->defaultControlPeriod;
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param thrForceInMsgPayload The commanded thruster forces
 */
THRArrayOnTimeCmdMsgPayload ThrFiringRemainderAlgorithm::update(const uint64_t callTime,
                                                                THRArrayCmdForceMsgPayload thrForceInMsgPayload) {
    double controlPeriod{};                        /* [s] control period */
    std::array<double, MAX_EFF_CNT> onTime{};      /* [s] array of commanded on time for thrusters */
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {}; /* [-] copy of the thruster on-time output message */
    /*! - The first time update() is called there is no information on the time step.
     *    Pick 2 seconds for the control period */
    if (this->prevCallTime == 0) {
        controlPeriod = this->defaultControlPeriod;
    } else {
        /*! - compute control time period Delta_t */
        controlPeriod = static_cast<double>(callTime - this->prevCallTime) * NANO2SEC;
    }

    this->prevCallTime = callTime;

    /*! - Loop through thrusters */
    for (int i = 0; i < this->numThrusters; i++) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (this->thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrForceInMsgPayload.thrForce[i] += this->maxThrust[i];
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrForceInMsgPayload.thrForce[i] < 0.0) {
            thrForceInMsgPayload.thrForce[i] = 0.0;
        }

        /*! - Compute T_on from thrust request, max thrust, and control period */
        onTime[i] = thrForceInMsgPayload.thrForce[i] / this->maxThrust[i] * controlPeriod;
        /*! - Add in remainder from the last control step */
        onTime[i] += this->pulseRemainder[i] * this->thrMinFireTime;
        /*! - Set pulse remainder to zero. Remainder now stored in onTime */
        this->pulseRemainder[i] = 0.0;

        /* Pulse remainder logic */
        if (onTime[i] < this->thrMinFireTime) {
            /*! - If request is less than minimum pulse time zero onTime an store remainder */
            this->pulseRemainder[i] = onTime[i] / this->thrMinFireTime;
            onTime[i] = 0.0;
        } else if (onTime[i] >= controlPeriod) {
            /*! - If request is greater than control period then oversaturate onTime */
            onTime[i] = 1.1 * controlPeriod;
        }

        /*! - Set the output data for each thruster */
        thrOnTimeOut.OnTimeRequest[i] = onTime[i];
    }

    return thrOnTimeOut;
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainderAlgorithm::setThrMinFireTime(const double thrMinFireTime) {
    if (thrMinFireTime < 0.0) {
        throw std::invalid_argument("ThrFiringRemainderAlgorithm::thrMinFireTime cannot be < 0.0");
    }
    this->thrMinFireTime = thrMinFireTime;
}

/*! Getter method for thrMinFireTime.
 @return const double
*/
double ThrFiringRemainderAlgorithm::getThrMinFireTime() const { return this->thrMinFireTime; }

/*! Setter method for thrustPulsingRegime.
 @return void
 @param thrustPulsingRegime
*/
void ThrFiringRemainderAlgorithm::setThrustPulsingRegime(const ThrustPulsingRegime thrustPulsingRegime) {
    this->thrustPulsingRegime = thrustPulsingRegime;
}

/*! Getter method for thrustPulsingRegime.
 @return ThrustPulsingRegime
*/
ThrustPulsingRegime ThrFiringRemainderAlgorithm::getThrustPulsingRegime() const { return this->thrustPulsingRegime; }

/*! Setter method for defaultControlPeriod.
 @return void
 @param defaultControlPeriod
*/
void ThrFiringRemainderAlgorithm::setDefaultControlPeriod(const double defaultControlPeriod) {
    if (defaultControlPeriod <= 0.0) {
        throw std::invalid_argument("ThrFiringRemainderAlgorithm::defaultControlPeriod cannot be <= 0.0");
    }
    this->defaultControlPeriod = defaultControlPeriod;
}

/*! Getter method for defaultControlPeriod.
 @return const double
*/
double ThrFiringRemainderAlgorithm::getDefaultControlPeriod() const { return this->defaultControlPeriod; }
