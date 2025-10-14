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

#include "thrFiringRemainder_C.h"
#include "thrFiringRemainderAlgorithm_C.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainderC::reset(uint64_t callTime)
{
	// check if the required input messages are included
	if (!this->thrConfInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
	}
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

	/*! - read in the support messages */
    const THRArrayConfigMsgPayload localThrusterData = this->thrConfInMsg();
    ::reset(&this->algorithmState, callTime, localThrusterData);
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainderC::updateState(uint64_t callTime) {
    const THRArrayCmdForceMsgPayload thrForceIn = this->thrForceInMsg();

    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = ::updateState(&this->algorithmState, callTime, thrForceIn);

    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainderC::setThrMinFireTime(const double thrMinFireTime) {
    this->algorithmState.thrMinFireTime = thrMinFireTime;
}

/*! Getter method for thrMinFireTime.
 @return const double
*/
double ThrFiringRemainderC::getThrMinFireTime() const {
	return this->algorithmState.thrMinFireTime;
}

/*! Setter method for baseThrustState.
 @return void
 @param baseThrustState
*/
void ThrFiringRemainderC::setBaseThrustState(const int baseThrustState) {
    this->algorithmState.baseThrustState = baseThrustState;
}

/*! Getter method for baseThrustState.
 @return const int
*/
int ThrFiringRemainderC::getBaseThrustState() const {
	return this->algorithmState.baseThrustState;
}

/*! Setter method for defaultControlPeriod.
 @return void
 @param defaultControlPeriod
*/
void ThrFiringRemainderC::setDefaultControlPeriod(const double defaultControlPeriod) {
    this->algorithmState.defaultControlPeriod = defaultControlPeriod;
}

/*! Getter method for defaultControlPeriod.
 @return const double
*/
double ThrFiringRemainderC::getDefaultControlPeriod() const {
	return this->algorithmState.defaultControlPeriod;
}
