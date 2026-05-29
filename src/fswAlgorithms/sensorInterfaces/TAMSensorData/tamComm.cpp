// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "tamComm.h"
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <math.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TamComm::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->tamInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: tamComm.tamInMsg wasn't connected.");
    }

    if (fabs(m33Determinant(RECAST3X3 this->dcm_BS) - 1.0) > 1e-10) {
        this->bskLogger.bskLog(BSK_WARNING, "dcm_BS is set to zero values.");
    }

    return;
}

/*! This method takes the sensor data from the magnetometers and
 converts that information to the format used by the TAM nav.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TamComm::updateState(uint64_t callTime) {
    // read input msg
    TAMSensorMsgPayload localInput = this->tamInMsg();

    m33MultV3(RECAST3X3 this->dcm_BS, localInput.tam_S, this->tamLocalOutput.tam_B);

    /*! - Write aggregate output into output message */
    this->tamOutMsg.write(tamLocalOutput, moduleID, callTime);

    return;
}
