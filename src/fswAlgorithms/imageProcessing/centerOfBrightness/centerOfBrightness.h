// SPDX-License-Identifier: ISC
// Copyright (c) 2018, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _IMAGE_PROC_COB_H_
#define _IMAGE_PROC_COB_H_

#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <memory>

#include <architecture/msgPayloadDef/RegionOfInterestMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
#include <architecture/msgPayloadDef/CenterOfBrightnessDiagnosticMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>

#include "centerOfBrightnessAlgorithm.h"
#include "imageReader/imageReaderInterface.h"

/*! @brief visual object tracking using center of brightness detection */
class CenterOfBrightness : public SysModel {
   public:
    explicit CenterOfBrightness(std::shared_ptr<ImageReaderInterface> imageReaderInstance);
    ~CenterOfBrightness() override;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    void setRelativeBrightnessIncreaseThreshold(double increaseThreshold);
    double getRelativeBrightnessIncreaseThreshold() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;
    void setCameraID(int32_t id);
    int32_t getCameraID() const;

    Message<OpNavCOBMsgPayload> opnavCOBOutMsg;  //!< The name of the OpNav center of brightness output message
    Message<CenterOfBrightnessDiagnosticMsgPayload> centerOfBrightnessDiagnosticOutMsg;
    ReadFunctor<RegionOfInterestMsgPayload> roiInMsg;  //!< Region of interest input message
    BSKLogger bskLogger;                               //!< -- BSK Logging

   private:
    CenterOfBrightnessAlgorithm algorithm{};
    std::shared_ptr<ImageReaderInterface> imageReader;  //!< shared ownership with Python/SWIG
    int32_t cameraID{};
};

#endif
