#include "thrFiringRemainder.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::reset(uint64_t callTime)
{
	// check if the required input messages are included
	if (!this->thrConfInMsg.isLinked()) {
		this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
	}
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

    /*! - read in the support messages */
    THRArrayConfigMsgPayload localThrusterData = this->thrConfInMsg();
    this->algorithm.reset(localThrusterData);
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void ThrFiringRemainder::updateState(uint64_t callTime) {
    THRArrayCmdForceMsgPayload thrForceIn = this->thrForceInMsg();

    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = this->algorithm.update(callTime, thrForceIn);
    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainder::setThrMinFireTime(const double thrMinFireTime) {
    this->algorithm.setThrMinFireTime(thrMinFireTime);
}

/*! Getter method for thrMinFireTime.
 @return const double
*/
double ThrFiringRemainder::getThrMinFireTime() const {
	return this->algorithm.getThrMinFireTime();
}

/*! Setter method for baseThrustState.
 @return void
 @param baseThrustState
*/
void ThrFiringRemainder::setBaseThrustState(const int baseThrustState) {
    this->algorithm.setBaseThrustState(baseThrustState);
}

/*! Getter method for baseThrustState.
 @return const int
*/
int ThrFiringRemainder::getBaseThrustState() const {
	return this->algorithm.getBaseThrustState();
}

/*! Setter method for defaultControlPeriod.
 @return void
 @param defaultControlPeriod
*/
void ThrFiringRemainder::setDefaultControlPeriod(const double defaultControlPeriod) {
    this->algorithm.setDefaultControlPeriod(defaultControlPeriod);
}

/*! Getter method for defaultControlPeriod.
 @return const double
*/
double ThrFiringRemainder::getDefaultControlPeriod() const {
	return this->algorithm.getDefaultControlPeriod();
}
