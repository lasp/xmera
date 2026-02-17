// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_MESSAGE_H
#define FILTER_MESSAGE_H

#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

/*! @brief structure for filter-states output from a filter*/
typedef struct
    //@cond DOXYGEN_IGNORE
    FilterMsgPayload
//@endcond
{
    double timeTag;                                       //!< [s] Current time of validity for output
    int numberOfStates;                                   //!< [-] Number of scalar states in the state vector
    double covar[MAX_STATES_VECTOR * MAX_STATES_VECTOR];  //!< [-] Current covariance of the filter
    double state[MAX_STATES_VECTOR];                      //!< [-] Current estimated state of the filter
    double stateError[MAX_STATES_VECTOR];  //!< [-] Current deviation of the state from the reference state
} FilterMsgPayload;

#endif /* FILTER_MESSAGE_H */
