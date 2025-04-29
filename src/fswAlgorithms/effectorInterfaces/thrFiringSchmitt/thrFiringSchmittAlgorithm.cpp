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

#include "fswAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmittAlgorithm.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringSchmittAlgorithm::reset(uint64_t callTime,
                                      THRArrayConfigMsgPayload const & thrusterConfigPayload)
{
	this->prevCallTime = 0;

    /*! - store the number of installed thrusters */
	this->numThrusters = thrusterConfigPayload.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
	for(uint32_t i=0; i<this->numThrusters; i++) {
		this->maxThrust[i] = thrusterConfigPayload.thrusters[i].maxThrust;
		this->lastThrustState[i] = BOOL_FALSE;
	}
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
THRArrayOnTimeCmdMsgPayload ThrFiringSchmittAlgorithm::update(uint64_t callTime,
                                                              THRArrayCmdForceMsgPayload& thrForceIn)
{
	int 				i;
	double 				level;					/* [-] duty cycle fraction */
	double				controlPeriod;			/* [s] control period */
	double				onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {};       /* -- copy of the thruster on-time output message */

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
	if(this->prevCallTime == 0) {
		this->prevCallTime = callTime;

		for(i = 0; i < this->numThrusters; i++) {
			thrOnTimeOut.OnTimeRequest[i] = (double)(this->baseThrustState) * 2.0;
		}

		return thrOnTimeOut;
	}

    /*! - compute control time period Delta_t */
	controlPeriod = ((double)(callTime - this->prevCallTime)) * NANO2SEC;
	this->prevCallTime = callTime;

    /*! - Loop through thrusters */
	for(i = 0; i < this->numThrusters; i++) {

        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
		if (this->baseThrustState == 1) {
			thrForceIn.thrForce[i] += this->maxThrust[i];
		}

        /*! - Do not allow thrust requests less than zero */
		if (thrForceIn.thrForce[i] < 0.0) {
			thrForceIn.thrForce[i] = 0.0;
		}
        /*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = thrForceIn.thrForce[i]/this->maxThrust[i]*controlPeriod;

        /*! - Apply Schmitt trigger logic */
		if (onTime[i] < this->thrMinFireTime) {
			/*! - Request is less than minimum fire time */
			level = onTime[i]/this->thrMinFireTime;
			if (level >= this->levelOn) {
				this->lastThrustState[i] = BOOL_TRUE;
				onTime[i] = this->thrMinFireTime;
			} else if (level <= this->levelOff) {
				this->lastThrustState[i] = BOOL_FALSE;
				onTime[i] = 0.0;
			} else if (this->lastThrustState[i] == BOOL_TRUE) {
				onTime[i] = this->thrMinFireTime;
			} else {
				onTime[i] = 0.0;
			}
		} else if (onTime[i] >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
			this->lastThrustState[i] = BOOL_TRUE;
			onTime[i] = 1.1*controlPeriod; // oversaturate to avoid numerical error
		} else {
			/*! - Request is greater than minimum fire time and less than control period */
			this->lastThrustState[i] = BOOL_TRUE;
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
double ThrFiringSchmittAlgorithm::getLevelOn() const {
    return this->levelOn;
}

/**
 * @brief Set the ON duty cycle fraction.
 * @param level The new ON duty cycle fraction to set.
 */
void ThrFiringSchmittAlgorithm::setLevelOn(double level) {
    this->levelOn = level;
}

/**
 * @brief Get the OFF duty cycle fraction.
 * @return double The current OFF duty cycle fraction.
 */
double ThrFiringSchmittAlgorithm::getLevelOff() const {
    return this->levelOff;
}

/**
 * @brief Set the OFF duty cycle fraction.
 * @param level The new OFF duty cycle fraction to set.
 */
void ThrFiringSchmittAlgorithm::setLevelOff(double level) {
    this->levelOff = level;
}

/**
 * @brief Get the minimum ON time for thrusters.
 * @return double The current minimum ON time in seconds.
 */
double ThrFiringSchmittAlgorithm::getThrMinFireTime() const {
    return this->thrMinFireTime;
}

/**
 * @brief Set the minimum ON time for thrusters.
 * @param time The new minimum ON time in seconds to set.
 */
void ThrFiringSchmittAlgorithm::setThrMinFireTime(double time) {
    this->thrMinFireTime = time;
}

/**
 * @brief Get the base thrust state.
 * @return int The current base thrust state (0 for off-pulsing, 1 for on-pulsing).
 */
int ThrFiringSchmittAlgorithm::getBaseThrustState() const {
    return this->baseThrustState;
}

/**
 * @brief Set the base thrust state.
 * @param state The new base thrust state to set (0 for off-pulsing, 1 for on-pulsing).
 */
void ThrFiringSchmittAlgorithm::setBaseThrustState(int state) {
    this->baseThrustState = state;
}
