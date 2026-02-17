// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EPHEM_NAV_CONVERTER_ALGORITHM_H
#define EPHEM_NAV_CONVERTER_ALGORITHM_H

#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <stdint.h>

/*! @brief The ephemNavConverter algorithm class.*/
class EphemNavConverterAlgorithm {
   public:
    EphemNavConverterAlgorithm() = default;   //!< Constructor
    ~EphemNavConverterAlgorithm() = default;  //!< Destructor

    NavTransMsgPayload update(uint64_t callTime, EphemerisMsgPayload ephemerisInMsg);
};

#endif
