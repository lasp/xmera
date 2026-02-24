// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_RES_MESSAGE_H
#define FILTER_RES_MESSAGE_H

#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

//!@brief Optical navigation measurement from filter containing post and pre fits
/*! This message contains the output from the filtering process given a specific measurement
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    FilterResidualsMsgPayload
//@endcond
{
    double timeTag;            //!< [s] Current time of validity for output
    bool valid;                //!< Quality of measurement if 1, invalid if 0
    int numberOfObservations;  //!< Number of observations in this message
    int sizeOfObservations;    //!< Size of observation vector in this message
    double observation[MAX_MEASUREMENT_VECTOR * MAX_MEASUREMENT_NUMBER];  //!< Measurement values processed
    double preFits[MAX_MEASUREMENT_VECTOR * MAX_MEASUREMENT_NUMBER];      //!< Measurement prefit residuals
    double postFits[MAX_MEASUREMENT_VECTOR * MAX_MEASUREMENT_NUMBER];     //!< Measurement postfit residuals
} FilterResidualsMsgPayload;

#endif /* FILTER_RES_MESSAGE_H */
