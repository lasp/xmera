// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _IMAGE_PROC_REGIONS_H_
#define _IMAGE_PROC_REGIONS_H_

#include "regionsOfInterestAlgorithm.h"

#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <Eigen/Core>

#include <architecture/msgPayloadDef/RegionsIdentifiedMsgPayload.h>
#include <architecture/_GeneralModuleFiles/sys_model.h>

/*! @brief visual object tracking using center of brightness detection */
class RegionsOfInterest : public SysModel {
   public:
    RegionsOfInterest();
    ~RegionsOfInterest() override;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    void setMaxRoiSeparation(int32_t pixelSeparation);
    int32_t getMaxRoiSeparation() const;
    void setWindowCenter(const Eigen::Vector2i& center);
    Eigen::Vector2i getWindowCenter() const;
    void setWindowSize(int32_t width, int32_t height);
    Eigen::Vector2i getWindowSize() const;
    void setImageSize(int32_t width, int32_t height);
    Eigen::Vector2i getImageSize() const;

    ReadFunctor<RegionsIdentifiedMsgPayload> roisInMsg;  //!< Input message containing detected regions
    Message<RegionOfInterestMsgPayload> regionOutMsg;    //!< Output message with identified ROI

   private:
    RegionsOfInterestAlgorithm algorithm{};  //!< Core algorithm instance
};

#endif
