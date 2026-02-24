// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*
    Attitude Tracking simple Module

 */

/* modify the path to reflect the new module names */
#include "simpleDeadband.h"
#include <math.h>

/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include <architecture/utilities/linearAlgebra.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SimpleDeadband::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->guidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: simpleDeadband.guidInMsg wasn't connected.");
    }
    this->wasControlOff = 1;
}

/*! This method parses the input data, checks if the deadband needs to be applied and outputs
 the guidance command with simples either zeroed (control OFF) or left unchanged (control ON)
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SimpleDeadband::updateState(uint64_t callTime) {
    /*! - Read the input message and set it as the output by default */
    this->attGuidOut = this->guidInMsg();

    /*! - Evaluate average simple in attitude and rates */
    this->attError = 4.0 * atan(v3Norm(this->attGuidOut.sigma_BR));
    this->rateError = v3Norm(this->attGuidOut.omega_BR_B);

    /*! - Check whether control should be ON or OFF */
    this->applyDBLogic_simpleDeadband();

    /*! - Write output guidance message and update module knowledge of control status*/
    this->attGuidOutMsg.write(&this->attGuidOut, this->moduleID, callTime);
    return;
}

/*! This method applies a two-level deadbanding logic (according to the current average simple compared with the set
 threshold) and decides whether control should be switched ON/OFF or not.
 @return void
 */
void SimpleDeadband::applyDBLogic_simpleDeadband() {
    uint32_t areErrorsBelowUpperThresh =
        (this->attError < this->outerAttThresh && this->rateError < this->outerRateThresh);
    uint32_t areErrorsBelowLowerThresh =
        (this->attError < this->innerAttThresh && this->rateError < this->innerRateThresh);

    if (areErrorsBelowUpperThresh) {
        if ((areErrorsBelowLowerThresh == 1) || ((areErrorsBelowLowerThresh == 0) && this->wasControlOff)) {
            /* Set simples to zero in order to turn off control */
            v3SetZero(this->attGuidOut.sigma_BR);
            v3SetZero(this->attGuidOut.omega_BR_B);
            this->wasControlOff = 1;
        } else {
            this->wasControlOff = 0;
        }
    } else {
        this->wasControlOff = 0;
    }
}
