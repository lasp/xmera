// SPDX-License-Identifier: ISC
// Copyright (c) 2018, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _IMAGE_PROC_COB_ALGORITHM_H_
#define _IMAGE_PROC_COB_ALGORITHM_H_

#include <Eigen/Core>
#include <memory>
#include <stdint.h>

#include "imageReader/imageReaderInterface.h"

/**
 * @brief Result struct for the center of brightness algorithm
 */
struct CenterOfBrightnessResult {
    Eigen::Vector2d centerOfBrightness = Eigen::Vector2d::Zero();
    int32_t pixelsFound{};
    double rollingAverageBrightness{};
    bool valid{};
    bool noPixelTrigger{false};
    bool notExceedingBrightnessIncreaseTrigger{false};
};

/**
 * @brief Bespoke input struct for the algorithm — no msg payload dependency
 */
struct CobRegionOfInterest {
    Eigen::Vector2i center = Eigen::Vector2i::Zero();
    Eigen::Vector2i size = Eigen::Vector2i::Zero();
};

/**
 * @brief Center of brightness detection algorithm
 *
 * Processes an image via an ImageReaderInterface to find the unweighted
 * center of brightness of non-zero pixels.
 */
class CenterOfBrightnessAlgorithm {
   public:
    CenterOfBrightnessAlgorithm();
    ~CenterOfBrightnessAlgorithm();

    CenterOfBrightnessResult update(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    void reset();

    void setRelativeBrightnessIncreaseThreshold(double increaseThreshold);
    double getRelativeBrightnessIncreaseThreshold() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;

   private:
    CenterOfBrightnessResult findCob(const CobRegionOfInterest& roi, ImageReaderInterface& imageReader);
    std::pair<Eigen::Vector2d, int32_t> computeCenterOfBrightness(
        const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels);
    double computeBrightnessIncrease(int32_t pixelsFound);
    void updateBrightnessHistory(double brightness);

    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> pixelBuffer =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    Eigen::VectorXd brightnessHistory{};           //!< [-] brightness history to be used for rolling average
    double relativeBrightnessIncreaseThreshold{};  //!< [-] minimum relative brightness increase (if less, invalidated)
    int32_t numberOfPointsBrightnessAverage{};  //!< [-] number of points to be used for rolling average of brightness
};

#endif
