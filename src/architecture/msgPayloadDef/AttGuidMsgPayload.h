// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ATT_GUID_MESSAGE_H
#define ATT_GUID_MESSAGE_H

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double sigma_BR[3];     //!<        Current attitude error estimate (MRPs) of B relative to R*/
    double omega_BR_B[3];   //!< [r/s]  Current body error estimate of B relateive to R in B frame compoonents */
    double omega_RN_B[3];   //!< [r/s]  Reference frame rate vector of the of R relative to N in B frame components */
    double domega_RN_B[3];  //!< [r/s2] Reference frame inertial body acceleration of R relative to N in B frame
                            //!< components */
} AttGuidMsgPayload;

#endif
