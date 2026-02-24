// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SC_MASS_PROPS_MESSAGE_H
#define SC_MASS_PROPS_MESSAGE_H

/*! @brief This structure is used in the messaging system to communicate what the mass
 properties of the vehicle are currently.*/
typedef struct {
    double massSC;            //!< kg   Current spacecraft mass
    double c_B[3];            //!< m    Center of mass of spacecraft with respect to point B
    double ISC_PntB_B[3][3];  //!< kgm2 Inertia tensor of spacecraft (relative to body)
} SCMassPropsMsgPayload;

#endif
