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
/*
    Thrust Firing Remainder

 */

#include "fswAlgorithms/effectorInterfaces/thrFiringRemainder/thrFiringRemainder.h"
#include "architecture/utilities/macroDefinitions.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::reset(uint64_t callTime)
{
	THRArrayConfigMsgPayload   localThrusterData;     /* local copy of the thruster data message */



	// check if the required input messages are included
	if (!this->thrConfInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
	}
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }
    /*! - read in the support messages */
    localThrusterData = this->thrConfInMsg();
    this->algorithm.reset(callTime, localThrusterData);
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::updateState(uint64_t callTime)
{
    THRArrayCmdForceMsgPayload thrForceIn = this->thrForceInMsg();          /* [-] copy of the thruster force input message */
    THRArrayConfigMsgPayload thrConfigInMsg = this->thrConfInMsg();
    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = this->algorithm.update(callTime,thrForceIn,thrConfigInMsg);       /* [-] copy of the thruster on-time output message */
    this->onTimeOutMsg.write(&thrOnTimeOut,this->moduleID,callTime);



}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainder::setThrMinFireTime(const double thrMinFireTime){this->algorithm.setThrMinFireTime(thrMinFireTime);}

/*! Getter method for thrMinFireTime.
 @return const double
*/
const double& ThrFiringRemainder::getThrMinFireTime() const{return this->algorithm.getThrMinFireTime();}

/*! Setter method for baseThrustState.
 @return void
 @param baseThrustState
*/
void ThrFiringRemainder::setBaseThrustState(const int baseThrustState){this-> algorithm.setBaseThrustState(baseThrustState);}

/*! Getter method for baseThrustState.
 @return const int
*/
const int& ThrFiringRemainder::getBaseThrustState() const{return this->algorithm.getBaseThrustState();}

/*! Setter method for defaultControlPeriod.
 @return void
 @param defaultControlPeriod
*/
void ThrFiringRemainder::setDefaultControlPeriod(const double defaultControlPeriod){this->algorithm.setDefaultControlPeriod(defaultControlPeriod);}

/*! Getter method for defaultControlPeriod.
 @return const double
*/
const double& ThrFiringRemainder::getDefaultControlPeriod() const{return this->algorithm.getDefaultControlPeriod();}