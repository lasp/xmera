// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef NAV_ATT_MESSAGE_H
#define NAV_ATT_MESSAGE_H

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double timeTag;          //!< [s]   Current vehicle time-tag associated with measurements*/
    double sigma_BN[3];      //!<       Current spacecraft attitude (MRPs) of body relative to inertial */
    double omega_BN_B[3];    //!< [r/s] Current spacecraft angular velocity vector of body frame B relative to inertial
                             //!< frame N, in B frame components
    double vehSunPntBdy[3];  //!<       Current sun pointing vector in body frame
} NavAttMsgPayload;

#endif
