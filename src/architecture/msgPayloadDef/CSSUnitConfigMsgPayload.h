// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CSS_UNIT_MESSAGE_H
#define CSS_UNIT_MESSAGE_H

/*! @brief Structure used to contain the configuration information for
 each sun sensor*/
typedef struct {
    double nHat_B[3];  //!< [-] CSS unit normal expressed in structure
    double
        CBias;  //!< [W]  Individual calibration coefficient bias for CSS.  If all CSS have the same gain, then this is
                //!< set to 1.0. If one CSS has a 10% stronger response for the same input, then the value would be 1.10
} CSSUnitConfigMsgPayload;

#endif
