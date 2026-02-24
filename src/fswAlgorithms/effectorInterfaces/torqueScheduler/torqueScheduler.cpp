// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "torqueScheduler.h"
#include <architecture/utilities/macroDefinitions.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void TorqueScheduler::reset(uint64_t callTime) {
    if (!this->motorTorque1InMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "torqueScheduler.motorTorque1InMsg wasn't connected.");
    }
    if (!this->motorTorque2InMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "torqueScheduler.motorTorque2InMsg wasn't connected.");
    }

    this->t0 = callTime;
}

/*! This method computes the control torque to the solar array drive based on a PD control law
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void TorqueScheduler::updateState(uint64_t callTime) {
    /*! - Create and assign buffer messages */
    ArrayMotorTorqueMsgPayload motorTorque1In = this->motorTorque1InMsg();
    ArrayMotorTorqueMsgPayload motorTorque2In = this->motorTorque2InMsg();
    ArrayMotorTorqueMsgPayload motorTorqueOut = {};
    ArrayEffectorLockMsgPayload effectorLockOut = {};

    /*! compute current time from Reset call */
    double t = ((callTime - this->t0) * NANO2SEC);

    /*! populate output torque msg */
    motorTorqueOut.motorTorque[0] = motorTorque1In.motorTorque[0];
    motorTorqueOut.motorTorque[1] = motorTorque2In.motorTorque[0];

    switch (this->lockFlag) {
        case 0:
            effectorLockOut.effectorLockFlag[0] = 0;
            effectorLockOut.effectorLockFlag[1] = 0;
            break;

        case 1:
            if (t > this->tSwitch) {
                effectorLockOut.effectorLockFlag[0] = 1;
                effectorLockOut.effectorLockFlag[1] = 0;
            } else {
                effectorLockOut.effectorLockFlag[0] = 0;
                effectorLockOut.effectorLockFlag[1] = 1;
            }
            break;

        case 2:
            if (t > this->tSwitch) {
                effectorLockOut.effectorLockFlag[0] = 0;
                effectorLockOut.effectorLockFlag[1] = 1;
            } else {
                effectorLockOut.effectorLockFlag[0] = 1;
                effectorLockOut.effectorLockFlag[1] = 0;
            }
            break;

        case 3:
            effectorLockOut.effectorLockFlag[0] = 1;
            effectorLockOut.effectorLockFlag[1] = 1;
            break;

        default:
            this->bskLogger.bskLog(BSK_ERROR, "Error: torqueScheduler.lockFlag has to be an integer between 0 and 3.");
    }

    /* write output messages */
    this->motorTorqueOutMsg.write(&motorTorqueOut, this->moduleID, callTime);
    this->effectorLockOutMsg.write(&effectorLockOut, this->moduleID, callTime);
}
