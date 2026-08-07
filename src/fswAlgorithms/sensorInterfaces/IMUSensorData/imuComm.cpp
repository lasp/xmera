// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "imuComm.h"

#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>

/*! This method resets the module.
 @return void
 @param configData The configuration data associated with the OD filter
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void ImuComm::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->imuComInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: imuComm.imuComInMsg wasn't connected.");
    }
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the IMU nav.
 @return void
 @param configData The configuration data associated with the IMU interface
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void ImuComm::updateState(uint64_t callTime) {
    // read imu com msg
    IMUSensorMsgPayload LocalInput = this->imuComInMsg();
    IMUSensorBodyMsgPayload outMsgBuffer = {};

    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.DVFramePlatform, outMsgBuffer.DVFrameBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.AccelPlatform, outMsgBuffer.AccelBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.DRFramePlatform, outMsgBuffer.DRFrameBody);
    m33MultV3(RECAST3X3 this->dcm_BP, LocalInput.AngVelPlatform, outMsgBuffer.AngVelBody);

    this->imuSensorOutMsg.write(outMsgBuffer, moduleID, callTime);

    return;
}
