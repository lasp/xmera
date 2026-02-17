// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef epochSimMsg_h
#define epochSimMsg_h

/*! @brief Structure used to define the the epoch date and time message */
typedef struct {
    int year;        //!< year, integer
    int month;       //!< month, integer
    int day;         //!< day, integer
    int hours;       //!< hours, integer
    int minutes;     //!< minutes, integer
    double seconds;  //!< seconds, double
} EpochMsgPayload;

#endif /* epochSimMsg_h */
