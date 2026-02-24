// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_SIMPLEPOWERSINK_H
#define XMERA_SIMPLEPOWERSINK_H

#include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>

/*! @brief simple power sink class */
class SimplePowerSink : public PowerNodeBase {
   public:
    SimplePowerSink();
    ~SimplePowerSink();

   private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload* powerUsageMsg);
};

#endif  // XMERA_SIMPLEPOWERSINK_H
