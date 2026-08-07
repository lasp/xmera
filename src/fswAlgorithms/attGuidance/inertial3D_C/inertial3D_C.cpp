// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*
    Inertial 3D Spin Module

 */

/* modify the path to reflect the new module names */
#include "inertial3D_C.h"

/* Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory. */
#include <architecture/utilities/linearAlgebra.h>

/*! Generate attitude reference associated with Intertial 3D Pointing.  In this case this is a fixed attitude
    with zero angular rate and acceleration vectors
 @return void
 @param attRefOut Output message
 */
static void computeInertialPointingReference(double sigma_R0N[3], AttRefMsgPayload* attRefOut) {
    v3Copy(sigma_R0N, attRefOut->sigma_RN);
    v3SetZero(attRefOut->omega_RN_N);
    v3SetZero(attRefOut->domega_RN_N);
}

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D_C::updateState(uint64_t callTime) {
    AttRefMsgPayload attRefOut = {}; /* output message structure */

    /*! - Compute and store output message */
    computeInertialPointingReference(this->sigma_R0N, &attRefOut);

    /*! - Write output message */
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);

    return;
}
