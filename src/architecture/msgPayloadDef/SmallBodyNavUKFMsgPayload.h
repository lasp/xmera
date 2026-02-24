// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SMALLBODY_NAVUKF_MESSAGE_H
#define SMALLBODY_NAVUKF_MESSAGE_H

/*! @brief Structure used to define the output of the sub-module.  This is the same
    output message that is used by all sub-modules in the module folder. */
typedef struct {
    double state[9];     //!< [units] state
    double covar[9][9];  //!< [units] covariance
} SmallBodyNavUKFMsgPayload;

#endif
