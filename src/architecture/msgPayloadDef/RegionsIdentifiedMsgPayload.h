#ifndef REGIONS_IDENTIFIED_H
#define REGIONS_IDENTIFIED_H

#include "RegionOfInterestMsgPayload.h"
#include "definitions.h"

/*! @brief Regions of interest extracted by camera */
typedef struct {
    RegionOfInterestMsgPayload regions[MAX_NUMBER_REGIONS];  //!< [ROI] array of regions
} RegionsIdentifiedMsgPayload;

#endif
