// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "stComm.h"

#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.h>

/*! This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void StComm::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->stSensorInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: stComm.stSensorInMsg wasn't connected.");
    }
}

/*! This method takes the raw sensor data from the star tracker and
 converts that information to the format used by the ST nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void StComm::updateState(uint64_t callTime) {
    double dcm_CN[3][3]; /* dcm, inertial to case frame */
    double dcm_BN[3][3]; /* dcm, inertial to body frame */

    // read input msg
    STSensorMsgPayload localInput = this->stSensorInMsg();

    EP2C(localInput.qInrtl2Case, dcm_CN);
    m33MultM33(RECAST3X3 this->dcm_BP, dcm_CN, dcm_BN);

    STAttMsgPayload attOutBuffer = {};
    C2MRP(dcm_BN, attOutBuffer.MRP_BdyInrtl);
    attOutBuffer.timeTag = localInput.timeTag;

    this->stAttOutMsg.write(attOutBuffer, this->moduleID, callTime);

    return;
}
