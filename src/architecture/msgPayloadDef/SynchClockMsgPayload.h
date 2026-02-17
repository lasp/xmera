// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SYNC_CLOCK_MESSAGE_H
#define SYNC_CLOCK_MESSAGE_H

//! @brief Output diagnostic structure used for analyzing how the synch is performing.
typedef struct {
    double initTimeDelta;     //!< s Time remaining in synch frame on arrival
    double finalTimeDelta;    //!< s Time remaining in synch frame on departure
    uint64_t overrunCounter;  //!< (-) Indicator of how many times we've missed the synch frame
} SynchClockMsgPayload;

#endif
