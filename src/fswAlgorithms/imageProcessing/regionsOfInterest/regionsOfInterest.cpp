// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "regionsOfInterest.h"

/*! Module constructor */
RegionsOfInterest::RegionsOfInterest() = default;

/*! Module destructor */
RegionsOfInterest::~RegionsOfInterest() = default;

/**
 * @brief Performs a complete reset of the module
 *
 * Resets the underlying algorithm to its initial state.
 *
 * @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void RegionsOfInterest::reset(uint64_t currentSimNanos) { this->algorithm.reset(); }

/**
 * @brief Updates the module state and processes detected regions
 *
 * This method performs the complete processing workflow:
 * 1. Reads the input message containing an array of detected regions
 * 2. Converts the message payload format to RegionOfInterest structures
 * 3. Calls the algorithm's update method to identify the key ROI
 * 4. Converts the result back to message payload format
 * 5. Writes the output message with timing information
 *
 * @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void RegionsOfInterest::updateState(uint64_t currentSimNanos) {
    // Read input message containing detected regions
    auto regionsMsgPayload = this->roisInMsg();

    // Convert message payload to algorithm structures
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regionsInput{};
    for (size_t index = 0; index < MAX_NUMBER_REGIONS; ++index) {
        RegionOfInterest regionInput{};
        regionInput.timeTag = regionsMsgPayload.timeTag[index];
        regionInput.regionCenter << regionsMsgPayload.centerX[index], regionsMsgPayload.centerY[index];
        regionInput.regionSize << regionsMsgPayload.width[index], regionsMsgPayload.height[index];
        regionInput.centerOfBrightness << regionsMsgPayload.centerX[index], regionsMsgPayload.centerY[index];
        regionInput.numberOfPixels = regionsMsgPayload.numberOfPixels[index];
        regionsInput.at(index) = regionInput;
    }

    // Process regions through algorithm
    auto [timeTag, regionCenter, regionSize, centerOfBrightness, numberOfPixels] = this->algorithm.update(regionsInput);
    // Convert result back to message payload format
    RegionOfInterestMsgPayload regionOutMsgPayload{.timeTag = timeTag,
                                                   .centerX = regionCenter.x(),
                                                   .centerY = regionCenter.y(),
                                                   .width = regionCenter.y(),
                                                   .height = regionSize.y(),
                                                   .numberOfPixels = numberOfPixels,
                                                   .centerOfBrightnessX = centerOfBrightness.x(),
                                                   .centerOfBrightnessY = centerOfBrightness.y()};

    // Write output message
    this->regionOutMsg.write(regionOutMsgPayload, this->moduleID, currentSimNanos);
}

/**
 * @brief Sets the maximum pixel separation for region merging
 *
 * Delegates to the underlying algorithm.
 *
 * @param pixelSeparation Maximum separation distance in pixels
 */
void RegionsOfInterest::setMaxRoiSeparation(const int32_t pixelSeparation) {
    this->algorithm.setMaxRoiSeparation(pixelSeparation);
}

/**
 * @brief Gets the current maximum ROI separation
 *
 * Delegates to the underlying algorithm.
 *
 * @return int32_t Maximum separation in pixels
 */
int32_t RegionsOfInterest::getMaxRoiSeparation() const { return this->algorithm.getMaxRoiSeparation(); }

/**
 * @brief Sets the center point of the windowing mask
 *
 * Converts from generic Eigen::Vector2i to Eigen::Vector2i and delegates
 * to the underlying algorithm.
 *
 * @param center Center point coordinates as generic Eigen vector
 */
void RegionsOfInterest::setWindowCenter(const Eigen::Vector2i& center) {
    const Eigen::Vector2i windowCenter = {center.x(), center.y()};
    this->algorithm.setWindowCenter(windowCenter);
}

/**
 * @brief Gets the current window center
 *
 * Retrieves from algorithm and converts from Eigen::Vector2i to generic
 * Eigen::Vector2i for SWIG compatibility.
 *
 * @return Eigen::Vector2i Window center coordinates
 */
Eigen::Vector2i RegionsOfInterest::getWindowCenter() const {
    auto windowCenter = this->algorithm.getWindowCenter();
    Eigen::Vector2i center = {windowCenter.x(), windowCenter.y()};
    return center;
}

/**
 * @brief Sets the dimensions of the windowing mask
 *
 * Delegates to the underlying algorithm.
 *
 * @param width Window width in pixels
 * @param height Window height in pixels
 */
void RegionsOfInterest::setWindowSize(const int32_t width, const int32_t height) {
    this->algorithm.setWindowSize(width, height);
}

/**
 * @brief Gets the current window dimensions
 *
 * Retrieves from algorithm and converts from Eigen::Vector2i to generic
 * Eigen::Vector2i for SWIG compatibility.
 *
 * @return Eigen::Vector2i Window size as vector (width, height)
 */
Eigen::Vector2i RegionsOfInterest::getWindowSize() const {
    auto windowSize = this->algorithm.getWindowSize();
    Eigen::Vector2i size = {windowSize.x(), windowSize.y()};
    return size;
}

/**
 * @brief Sets the image size
 *
 * The image size defines the total number of x and y pixels.
 *
 * @param width X pixels
 * @param height Y pixels
 */
void RegionsOfInterest::setImageSize(const int32_t width, const int32_t height) {
    this->algorithm.setImageSize(width, height);
}

/**
 * @brief Gets the image size
 *
 * @return Eigen::Vector2i Image size as (x, y) in pixels
 */
Eigen::Vector2i RegionsOfInterest::getImageSize() const { return this->algorithm.getImageSize(); }
