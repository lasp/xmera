// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "dvGuidance.h"
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.h>
#include <architecture/utilities/macroDefinitions.h>

/*! @brief This resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvGuidance::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->burnDataInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: dvGuidance.burnDataInMsg wasn't connected.");
    }
    return;
}

/*! This method takes its own internal variables and creates an output attitude
    command to use for burn execution.  It also flags whether the burn should
    be happening or not.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DvGuidance::updateState(uint64_t callTime) {
    double dcm_BubN[3][3];             /* dcm, inertial to base burn frame */
    double dcm_ButN[3][3];             /* dcm, inertial to current burn frame */
    double dcm_ButBub[3][3];           /* dcm, rotating from base to current burn frame */
    double dvHat_N[3];                 /* unit vector, direction of delta velocity in the inertial frame */
    double bu2_N[3];                   /* vector, vector which becomes the BubN DCM's second basis vector */
    double burnTime;                   /* duration for which to thrust */
    double rotPRV[3];                  /* principle rotation vector about which to rotate during the burn */
    DvBurnCmdMsgPayload localBurnData; /* [-] input message container */
    AttRefMsgPayload attCmd = {};      /* [-] Output attitude command data to send */

    /*! - read in DV burn command input message */
    localBurnData = this->burnDataInMsg();

    /*! - evaluate DCM from inertial to the base Burn Frame */
    v3Normalize(localBurnData.dvInrtlCmd, dvHat_N);
    v3Copy(dvHat_N, dcm_BubN[0]);
    v3Cross(localBurnData.dvRotVecUnit, dvHat_N, bu2_N);
    v3Normalize(bu2_N, dcm_BubN[1]);
    v3Cross(dcm_BubN[0], dcm_BubN[1], dcm_BubN[2]);
    v3Normalize(dcm_BubN[2], dcm_BubN[2]);

    /*! - evaluate the time since the burn start time */
    burnTime = ((int64_t)callTime - (int64_t)localBurnData.burnStartTime) * NANO2SEC;

    /*! - evaluate the DCM from inertial to the current Burn frame.
     The current frame differs from the base burn frame via a constant 3-axis rotation */
    v3SetZero(rotPRV);
    rotPRV[2] = 1.0;
    v3Scale(burnTime * localBurnData.dvRotVecMag, rotPRV, rotPRV);
    PRV2C(rotPRV, dcm_ButBub);
    m33MultM33(dcm_ButBub, dcm_BubN, dcm_ButN);

    /*! - Compute the reference attitude */
    C2MRP(RECAST3X3 & dcm_ButN, attCmd.sigma_RN);
    /*! - Compute the reference frame angular rate vector */
    v3Scale(localBurnData.dvRotVecMag, dcm_ButN[2], attCmd.omega_RN_N);
    /*! - Zero the reference frame angular acceleration vector */
    v3SetZero(attCmd.domega_RN_N);

    /*! - Write the output message */
    this->attRefOutMsg.write(attCmd, this->moduleID, callTime);

    return;
}
