// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "centerOfBrightnessAlgorithm.h"

#include <iostream>

CenterOfBrightnessAlgorithm::CenterOfBrightnessAlgorithm() = default;

CenterOfBrightnessAlgorithm::~CenterOfBrightnessAlgorithm() = default;

/*! Reset algorithm state: clears brightness history */
void CenterOfBrightnessAlgorithm::reset() { this->brightnessHistory.resize(0); }

/*! Main entry point: delegates to findCob.
 @return CenterOfBrightnessResult
 @param roi Region of interest for windowing
 @param imageReader Image reader providing pixel data
 */
CenterOfBrightnessResult CenterOfBrightnessAlgorithm::update(const CobRegionOfInterest& roi,
                                                             ImageReaderInterface& imageReader) {
    CenterOfBrightnessResult result{};
    imageReader.getImageAsArray(roi.center, roi.size, *this->pixelBuffer);
    auto [coordinates, pixelsFound] = this->computeCenterOfBrightness(*this->pixelBuffer);

    if (pixelsFound > 0) {
        double brightnessIncrease = this->computeBrightnessIncrease(pixelsFound);
        result.noPixelTrigger = false;
        if (brightnessIncrease >= this->relativeBrightnessIncreaseThreshold) {
            result.valid = true;
            result.centerOfBrightness = coordinates;
            result.pixelsFound = pixelsFound;
            result.notExceedingBrightnessIncreaseTrigger = false;
        }
        result.rollingAverageBrightness = this->brightnessHistory.mean();
    }
    return result;
}

/*! Compute the unweighted centroid of non-zero pixels. (0,0) is treated as sentinel / invalid.
 @return pair of centroid coordinates and count of valid pixels
 @param pixels Array of pixel coordinates from image reader
 */
std::pair<Eigen::Vector2d, int32_t> CenterOfBrightnessAlgorithm::computeCenterOfBrightness(
    const std::array<Eigen::Vector2i, kMaxWindowSize>& pixels) {
    Eigen::Vector2d coordinates = Eigen::Vector2d::Zero();
    int32_t count = 0;
    for (const auto& pixel : pixels) {
        if (pixel.isZero()) continue;
        coordinates[0] += pixel[0];
        coordinates[1] += pixel[1];
        ++count;
    }
    if (count > 0) {
        coordinates /= count;
    }
    return {coordinates, count};
}

/*! Compute relative brightness increase from the rolling average.
 @return relative brightness increase
 @param pixelsFound Number of bright pixels found this timestep
 */
double CenterOfBrightnessAlgorithm::computeBrightnessIncrease(int32_t pixelsFound) {
    double averageBrightnessOld = 0.0;
    if (this->brightnessHistory.rows() > 0) {
        averageBrightnessOld = this->brightnessHistory.mean();
    }
    this->updateBrightnessHistory(static_cast<double>(pixelsFound));
    double averageBrightnessNew = this->brightnessHistory.mean();
    double brightnessIncrease = 0.0;
    if (averageBrightnessOld > 0.0) {
        brightnessIncrease = (averageBrightnessNew - averageBrightnessOld) / averageBrightnessOld;
    }
    return brightnessIncrease;
}

/*! Update brightness history by shifting back previous brightness values and updating most recent one
    @return void
    @param brightness total brightness of current time step
    */
void CenterOfBrightnessAlgorithm::updateBrightnessHistory(double brightness) {
    // increase vector size if it is not at its full size yet
    if (this->brightnessHistory.rows() < this->numberOfPointsBrightnessAverage) {
        this->brightnessHistory.conservativeResize(this->brightnessHistory.rows() + 1, 1);
    }
    // shift previous brightness values back (only if number of data points for rolling average is greater than 1)
    if (this->brightnessHistory.rows() > 1) {
        for (auto i = static_cast<int>(this->brightnessHistory.rows()) - 1; i > 0; --i) {
            this->brightnessHistory[i] = this->brightnessHistory[i - 1];
        }
    }
    // update most recent brightness value
    this->brightnessHistory[0] = brightness;
}

void CenterOfBrightnessAlgorithm::setRelativeBrightnessIncreaseThreshold(double increaseThreshold) {
    this->relativeBrightnessIncreaseThreshold = increaseThreshold;
}

double CenterOfBrightnessAlgorithm::getRelativeBrightnessIncreaseThreshold() const {
    return this->relativeBrightnessIncreaseThreshold;
}

void CenterOfBrightnessAlgorithm::setNumberOfPointsBrightnessAverage(int32_t rollingAverage) {
    this->numberOfPointsBrightnessAverage = rollingAverage;
}

int32_t CenterOfBrightnessAlgorithm::getNumberOfPointsBrightnessAverage() const {
    return this->numberOfPointsBrightnessAverage;
}
