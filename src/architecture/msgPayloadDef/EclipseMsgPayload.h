// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef eclipseSimMsg_h
#define eclipseSimMsg_h

//!@brief Eclipse shadow factor message definition.
typedef struct {
    double shadowFactor;  //!< Proportion of shadowing due to eclipse
} EclipseMsgPayload;

#endif /* eclipseSimMsg_h */
