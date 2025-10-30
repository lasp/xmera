#include "forceTorqueThrForceMapping.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void ForceTorqueThrForceMapping::reset(const uint64_t callTime) {
    if (!this->thrConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgPayload vehConfigInMsgBuffer = this->vehConfigInMsg();
    THRArrayConfigMsgPayload thrConfigInMsgBuffer = this->thrConfigInMsg();

    this->algorithm.reset(vehConfigInMsgBuffer, thrConfigInMsgBuffer);
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ForceTorqueThrForceMapping::updateState(const uint64_t callTime) {
    CmdTorqueBodyMsgPayload cmdTorqueInMsgBuffer{};
    CmdForceBodyMsgPayload cmdForceInMsgBuffer{};

    /* Check if torque message is linked and read, zero out if not*/
    if (this->cmdTorqueInMsg.isLinked()) {
        cmdTorqueInMsgBuffer = this->cmdTorqueInMsg();
    }

    /* Check if force message is linked and read, zero out if not*/
    if (this->cmdForceInMsg.isLinked()) {
        cmdForceInMsgBuffer = this->cmdForceInMsg();
    }

    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer =
        this->algorithm.update(cmdTorqueInMsgBuffer, cmdForceInMsgBuffer);

    this->thrForceCmdOutMsg.write(&thrForceCmdOutMsgBuffer, this->moduleID, callTime);
}
