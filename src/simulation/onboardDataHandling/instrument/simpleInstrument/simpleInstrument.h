// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_SIMPLEINSTRUMENT_H
#define XMERA_SIMPLEINSTRUMENT_H

#include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>

/*! @brief simple instrument data handling class */
class SimpleInstrument : public DataNodeBase {
   public:
    SimpleInstrument();
    ~SimpleInstrument();

   private:
    void evaluateDataModel(DataNodeUsageMsgPayload* dataUsageMsg,
                           double currentTime);  //!< Sets the name and baud rate for the data in the output message.
};

#endif  // XMERA_SIMPLEINSTRUMENT_H
