// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _MODULE_TEMPLATE_MSG_H_
#define _MODULE_TEMPLATE_MSG_H_

/*! @brief Sample message payload struct for template module. */
typedef struct {
    double dataVector[3];  //!< [units] sample message vector
} ModuleTemplateMsgPayload;

#endif  // _MODULE_TEMPLATE_MSG_H_
