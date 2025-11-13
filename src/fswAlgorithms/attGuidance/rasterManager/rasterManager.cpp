/*
 Inertial 3D Spin Module

 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

#include "rasterManager.h"
#include <stdio.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include <architecture/utilities/linearAlgebra.h>

void RasterManager::reset(uint64_t callTime) {
    this->mnvrActive = 0;
    this->scanSelector = 0;
}

void RasterManager::updateState(uint64_t callTime) {
    double currentMnvrTime;
    this->scanSelector = this->scanSelector % this->numRasters;
    if (this->mnvrActive == 0) {
        this->mnvrStartTime = callTime;
        this->mnvrActive = 1;
    }
    currentMnvrTime = (callTime - this->mnvrStartTime) * 1E-9;
    if (currentMnvrTime < this->rasterTimes[this->scanSelector]) {
        v3Copy(&this->scanningAngles[3 * this->scanSelector], this->attOutSet.state);
        v3Copy(&this->scanningRates[3 * this->scanSelector], this->attOutSet.rate);
    } else {
        this->mnvrActive = 0;
        this->scanSelector += 1;

        char info[MAX_LOGGING_LENGTH];
        snprintf(info,
                 sizeof(info),
                 "Raster: %i. AngleSet = [%f, %f, %f], RateSet = [%f, %f, %f] ",
                 this->scanSelector,
                 this->attOutSet.state[0],
                 this->attOutSet.state[1],
                 this->attOutSet.state[2],
                 this->attOutSet.rate[0],
                 this->attOutSet.rate[1],
                 this->attOutSet.rate[2]);
        this->bskLogger.bskLog(BSK_INFORMATION, info);
    }

    this->attStateOutMsg.write(&this->attOutSet, this->moduleID, callTime);

    return;
}
