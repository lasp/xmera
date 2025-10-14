/*
 ISC License

 Copyright (c) 2025, University of Colorado at Boulder

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


#include "thrFiringRemainderAlgorithm_C.h"
#include "architecture/utilities/macroDefinitions.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param localThrusterData
 */

void reset(ThrFiringRemainderInternalState *moduleState, uint64_t callTime, const THRArrayConfigMsgPayload localThrusterData)
{
	moduleState->prevCallTime = 0;

    /*! - store the number of installed thrusters */
	moduleState->numThrusters = localThrusterData.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
	for(uint32_t i=0; i < moduleState->numThrusters; ++i) {
		moduleState->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		moduleState->pulseRemainder[i] = 0.0;
	}

    /*! - use default value of 2 seconds for control period of first call if not specified.
     * Control period (FSW rate) is computed dynamically for any subsequent calls.
     */
    moduleState->defaultControlPeriod = 0.0 == moduleState->defaultControlPeriod ?
										2.0 : moduleState->defaultControlPeriod;
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param thrForceIn
 */
THRArrayOnTimeCmdMsgPayload updateState(ThrFiringRemainderInternalState *moduleState,
                                        const uint64_t callTime,
                                        THRArrayCmdForceMsgPayload thrForceIn) {
	/*! - The first time update() is called there is no information on the time step.
	 *    Pick 2 seconds for the control period */
	double controlPeriod; /* [s] control period */
	if(moduleState->prevCallTime == 0) {
		controlPeriod = moduleState->defaultControlPeriod;
	} else {
		/*! - compute control time period Delta_t */
		controlPeriod = (double)(callTime - moduleState->prevCallTime) * NANO2SEC;
	}

	moduleState->prevCallTime = callTime;

	THRArrayOnTimeCmdMsgPayload thrOnTimeOut = {}; /* empty thruster on-time output message */

	/*! - Loop through thrusters */
	for(uint32_t i = 0; i < moduleState->numThrusters; i++) {

		/*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
		 needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
		 be set to the maximum thrust value */
		if (moduleState->baseThrustState == 1) {
			thrForceIn.thrForce[i] += moduleState->maxThrust[i];
		}

		/*! - Do not allow thrust requests less than zero */
		if (thrForceIn.thrForce[i] < 0.0) {
			thrForceIn.thrForce[i] = 0.0;
		}

		double onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
		/*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = thrForceIn.thrForce[i]/moduleState->maxThrust[i]*controlPeriod;
		/*! - Add in remainder from the last control step */
		onTime[i] += moduleState->pulseRemainder[i]*moduleState->thrMinFireTime;
		/*! - Set pulse remainder to zero. Remainder now stored in onTime */
		moduleState->pulseRemainder[i] = 0.0;

		/* Pulse remainder logic */
		if(onTime[i] < moduleState->thrMinFireTime) {
			/*! - If request is less than minimum pulse time zero onTime an store remainder */
			moduleState->pulseRemainder[i] = onTime[i]/moduleState->thrMinFireTime;
			onTime[i] = 0.0;
		} else if (onTime[i] >= controlPeriod) {
			/*! - If request is greater than control period then oversaturate onTime */
			onTime[i] = 1.1*controlPeriod;
		}

		/*! - Set the output data for each thruster */
		thrOnTimeOut.OnTimeRequest[i] = onTime[i];
	}

	return thrOnTimeOut;
}