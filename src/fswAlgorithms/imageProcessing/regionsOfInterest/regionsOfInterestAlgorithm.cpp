// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "regionsOfInterestAlgorithm.h"

/*! Module constructor */
RegionsOfInterestAlgorithm::RegionsOfInterestAlgorithm() = default;

/*! Module destructor */
RegionsOfInterestAlgorithm::~RegionsOfInterestAlgorithm() = default;

/**
 * @brief Performs a complete reset of the module
 *
 * Local module variables that retain time varying states between function calls
 * are reset to their default values. This method recomputes the window parameters.
 */
void RegionsOfInterestAlgorithm::reset() { this->computeWindow(); }

/**
 * @brief Main update method that processes regions and identifies the key ROI
 *
 * This method performs the complete processing pipeline:
 * 1. Applies windowing to filter out regions outside the detection area
 * 2. Orders the windowed regions by size (pixel count)
 * 3. Identifies the most relevant region of interest
 *
 * @param regions Input array of detected regions from image processing
 * @return RegionOfInterest The identified key region of interest
 */
RegionOfInterest RegionsOfInterestAlgorithm::update(
    const std::array<RegionOfInterest, MAX_NUMBER_REGIONS>& regions) const {
    RegionOfInterest outputRegion{};

    // Apply spatial windowing to filter regions
    auto const regionsWindowed = this->applyWindow(regions);

    // Order regions by pixel count (largest first)
    auto const windowedAndOrderedRegions = RegionsOfInterestAlgorithm::orderRegions(regionsWindowed);

    // Identify the key region of interest
    outputRegion = this->identifyRoi(windowedAndOrderedRegions);

    return outputRegion;
}

/**
 * @brief Applies windowing filter to input regions
 *
 * This method checks each region to see if its center of brightness falls within
 * the defined window bounds. Only regions within the window are retained.
 *
 * @param regions Input array of regions to filter
 * @return std::array<RegionOfInterest, MAX_NUMBER_REGIONS> Filtered regions within window bounds
 */
std::array<RegionOfInterest, MAX_NUMBER_REGIONS> RegionsOfInterestAlgorithm::applyWindow(
    const std::array<RegionOfInterest, MAX_NUMBER_REGIONS>& regions) const {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> checkedRegions{};
    size_t index = 0;

    // Check each region against window bounds
    for (auto const& region : regions) {
        if (this->regionInWindow(region)) {
            checkedRegions.at(index) = region;
            ++index;
        }
    }
    return checkedRegions;
}

/**
 * @brief Orders regions by pixel count in descending order
 *
 * Uses std::sort with a lambda comparator to arrange regions from largest
 * to smallest based on numberOfPixels.
 *
 * @param regions Input array of regions
 * @return std::array<RegionOfInterest, MAX_NUMBER_REGIONS> Sorted array with largest regions first
 */
std::array<RegionOfInterest, MAX_NUMBER_REGIONS> RegionsOfInterestAlgorithm::orderRegions(
    const std::array<RegionOfInterest, MAX_NUMBER_REGIONS>& regions) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> sorted = regions;
    // The comparator must be a strict weak ordering. A reflexive comparator (>=) causes
    // undefined behavior, and std::sort can then read data after the end of the range.
    std::ranges::sort(sorted.begin(), sorted.end(), [](const RegionOfInterest& a, const RegionOfInterest& b) {
        return a.numberOfPixels > b.numberOfPixels;
    });
    return sorted;
}

/**
 * @brief Checks if a region's center of brightness is within the window bounds
 *
 * A region is considered within the window if its center of brightness is strictly
 * inside the window rectangle (not on the boundary).
 *
 * @param region The region to check
 * @return bool True if center of brightness is within window bounds, false otherwise
 */
bool RegionsOfInterestAlgorithm::regionInWindow(const RegionOfInterest& region) const {
    Eigen::Vector2i centerOfBrightness = region.centerOfBrightness;
    return (windowPointTopLeft.x() < centerOfBrightness.x() && windowPointTopLeft.y() < centerOfBrightness.y() &&
            windowPointBottomRight.x() > centerOfBrightness.x() && windowPointBottomRight.y() > centerOfBrightness.y());
}

/**
 * @brief Computes the corner points of the windowing rectangle
 *
 * This method calculates the top-left and bottom-right corners based on the
 * window center and dimensions. If no window is defined (center is zero or
 * dimensions are zero), the window defaults to the entire image.
 *
 * The method validates that the computed window stays within image boundaries
 * and throws exceptions if the window extends outside.
 *
 * @throws std::invalid_argument If window extends beyond left edge
 * @throws std::invalid_argument If window extends beyond top edge
 * @throws std::invalid_argument If window extends beyond right edge
 * @throws std::invalid_argument If window extends beyond bottom edge
 */
void RegionsOfInterestAlgorithm::computeWindow() {
    // If no window is defined, use entire image
    if (this->windowCenter.isZero() || this->windowWidth == 0 || this->windowHeight == 0) {
        this->windowCenter.x() = this->imageSize.x() / 2;
        this->windowCenter.y() = this->imageSize.y() / 2;
        this->windowWidth = this->imageSize.x();
        this->windowHeight = this->imageSize.y();
    }
    // Compute window corners from center and size
    this->windowPointTopLeft.x() = this->windowCenter.x() - this->windowWidth / 2;
    this->windowPointTopLeft.y() = this->windowCenter.y() - this->windowHeight / 2;
    this->windowPointBottomRight.x() = this->windowCenter.x() + this->windowWidth / 2;
    this->windowPointBottomRight.y() = this->windowCenter.y() + this->windowHeight / 2;

    // Validate window boundaries
    if (windowPointTopLeft.x() < 0) {
        throw std::invalid_argument("Window went outside of the image to the left");
    }
    if (windowPointTopLeft.y() < 0) {
        throw std::invalid_argument("Window went outside of the image to the top");
    }
    if (windowPointBottomRight.x() > this->imageSize.x()) {
        throw std::invalid_argument("Window went outside of the image to the right");
    }
    if (windowPointBottomRight.y() > this->imageSize.y()) {
        throw std::invalid_argument("Window went outside of the image to the bottom");
    }
}

/**
 * @brief Identifies the key region of interest from an ordered array of regions
 *
 * This method implements the core region selection/merging logic:
 *
 * 1. If all regions are below the minimum detection threshold, returns empty region
 * 2. Calculates the barycenter (weighted center) of all valid regions
 * 3. Counts how many regions are close to this barycenter (within maxSeparation)
 * 4. If only one region is close to barycenter: returns the largest region
 * 5. If multiple regions are close: merges them into a composite region
 *    - Center of brightness becomes the barycenter
 *    - Total pixel count is sum of all regions
 *    - Region size is based on maxSeparation distance
 *
 * The barycentric approach handles cases where a single object has been split
 * into multiple detected regions due to image processing artifacts.
 *
 * @param regions Input array of regions, ordered by size (largest first)
 * @return RegionOfInterest The identified key region, or empty region if none found
 */
RegionOfInterest RegionsOfInterestAlgorithm::identifyRoi(
    const std::array<RegionOfInterest, MAX_NUMBER_REGIONS>& regions) const {
    RegionOfInterest keyRegion{};
    Eigen::Vector2i cobBarycenter = Eigen::Vector2i::Zero();
    auto threshold = this->minDetectionPixel;

    // Check if all regions are below detection threshold
    const auto all_zero = std::ranges::all_of(
        regions, [threshold](const RegionOfInterest& item) { return item.numberOfPixels <= threshold; });

    if (!all_zero) {
        std::array<RegionOfInterest, MAX_NUMBER_REGIONS> nonZeroRegions{};
        auto index = 0;
        for (auto const& region : regions) {
            if (region.numberOfPixels > threshold) {
                nonZeroRegions.at(index) = region;
            }
            ++index;
        }

        // Calculate weighted barycenter of all regions
        auto totalPixels = 0;
        for (auto const& region : nonZeroRegions) {
            cobBarycenter += region.numberOfPixels * region.centerOfBrightness;
            totalPixels += region.numberOfPixels;
        }
        cobBarycenter /= totalPixels;

        // Count regions close to barycenter
        auto numberOfRegionsCloseToBarycenter = 0;
        for (auto const& region : nonZeroRegions) {
            if (auto distanceToBarycenter = region.centerOfBrightness - cobBarycenter;
                distanceToBarycenter.norm() < this->maxSeparation) {
                ++numberOfRegionsCloseToBarycenter;
            }
        }

        // Decide whether to use single region or merged composite
        if (numberOfRegionsCloseToBarycenter < 2) {
            // Only one significant region - use it directly
            keyRegion = regions.at(0);
        } else {
            // Multiple close regions - create merged region at barycenter
            keyRegion.centerOfBrightness = cobBarycenter;
            keyRegion.regionCenter = cobBarycenter;
            keyRegion.numberOfPixels = totalPixels;
        }

        // Set region size based on maxSeparation (square inscribed in separation circle)
        keyRegion.regionSize = Eigen::Vector2i(std::floor(this->maxSeparation / std::sqrt(2)),
                                               std::floor(this->maxSeparation / std::sqrt(2)));
    }

    return keyRegion;
}

/**
 * @brief Sets the maximum pixel separation for region merging
 *
 * Regions whose centers of brightness are within this distance are considered
 * potentially part of the same object and may be merged.
 *
 * @param pixelSeparation Maximum separation distance in pixels
 */
void RegionsOfInterestAlgorithm::setMaxRoiSeparation(const int32_t pixelSeparation) {
    this->maxSeparation = pixelSeparation;
}

/**
 * @brief Gets the current maximum ROI separation setting
 *
 * @return int32_t Maximum separation in pixels
 */
int32_t RegionsOfInterestAlgorithm::getMaxRoiSeparation() const { return this->maxSeparation; }

/**
 * @brief Sets the camera ID for this algorithm instance
 *
 * @param id Camera identifier
 */
void RegionsOfInterestAlgorithm::setCameraId(int32_t id) { this->cameraId = id; }

/**
 * @brief Gets the current camera ID
 *
 * @return int32_t Camera identifier
 */
int32_t RegionsOfInterestAlgorithm::getCameraId() const { return this->cameraId; }

/**
 * @brief Sets the minimum detection size threshold
 *
 * Regions with fewer pixels than this threshold will be considered invalid
 * and will be filtered out during processing.
 *
 * @param pixels Minimum number of pixels required for valid detection
 */
void RegionsOfInterestAlgorithm::setMinimumDetectionSize(int32_t pixels) { this->minDetectionPixel = pixels; }

/**
 * @brief Gets the current minimum detection size
 *
 * @return int32_t Minimum number of pixels for detection
 */
int32_t RegionsOfInterestAlgorithm::getMinimumDetectionSize() const { return this->minDetectionPixel; }

/**
 * @brief Sets the center point of the windowing mask
 *
 * The window defines a rectangular region of the image where detections are
 * considered valid. Regions outside this window will be filtered out.
 *
 * @param center Center point coordinates (x, y) in pixels
 */
void RegionsOfInterestAlgorithm::setWindowCenter(const Eigen::Vector2i& center) { this->windowCenter = center; }

/**
 * @brief Gets the current window center coordinates
 *
 * @return Eigen::Vector2i Window center as (x, y) in pixels
 */
Eigen::Vector2i RegionsOfInterestAlgorithm::getWindowCenter() const { return this->windowCenter; }

/**
 * @brief Sets the image size
 *
 * The image size defines the total number of x and y pixels.
 *
 * @param width X pixels
 * @param height Y pixels
 */
void RegionsOfInterestAlgorithm::setImageSize(const int32_t width, const int32_t height) {
    this->imageSize << width, height;
}

/**
 * @brief Gets the image size
 *
 * @return Eigen::Vector2i Image size as (x, y) in pixels
 */
Eigen::Vector2i RegionsOfInterestAlgorithm::getImageSize() const { return this->imageSize; }

/**
 * @brief Sets the dimensions of the windowing mask
 *
 * The window is a rectangle centered at windowCenter with these dimensions.
 *
 * @param width Window width in pixels
 * @param height Window height in pixels
 */
void RegionsOfInterestAlgorithm::setWindowSize(const int32_t width, const int32_t height) {
    this->windowWidth = width;
    this->windowHeight = height;
}

/**
 * @brief Gets the current window dimensions
 *
 * @return Eigen::Vector2i Window size as (width, height) in pixels
 */
Eigen::Vector2i RegionsOfInterestAlgorithm::getWindowSize() const {
    Eigen::Vector2i center = {this->windowWidth, this->windowHeight};
    return center;
}
