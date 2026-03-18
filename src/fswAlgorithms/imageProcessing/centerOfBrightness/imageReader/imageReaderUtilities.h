#ifndef _IMAGE_READER_UTILITIES_H_
#define _IMAGE_READER_UTILITIES_H_

#include "imageReaderInterface.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

/*! Filter pixel coordinates to those within the specified window and write to output.
 *  Unused slots are filled with (0,0).
 */
inline void extractImageWindow(const std::vector<Eigen::Vector2i>& data,
                               const Eigen::Vector2i& center,
                               const Eigen::Vector2i& windowSize,
                               std::array<Eigen::Vector2i, kMaxWindowSize>& output) {
    // Verify that the window size matches the array size
    if (windowSize[0] * windowSize[1] > kMaxWindowSize) {
        throw std::invalid_argument("Window size exceeded the max allowable " + std::to_string(kMaxWindowSize) +
                                    " pixels");
    }
    // Calculate the top-left corner of the window
    const int startX = center[0] - windowSize[0] / 2;
    const int startY = center[1] - windowSize[1] / 2;

    output.fill(Eigen::Vector2i::Zero());
    std::size_t idx = 0;
    // Filter to only non-zero pixel coordinates that fall within the window
    for (const auto& coordinate : data) {
        if (coordinate[0] >= startX && coordinate[0] < startX + windowSize[0] && coordinate[1] >= startY &&
            coordinate[1] < startY + windowSize[1]) {
            output[idx] = coordinate;
            ++idx;
        }
    }
}

/*! Convert a BGR cv::Mat to a vector of non-zero pixel coordinates after
 *  grayscale conversion, box blur, and binary thresholding.
 */
inline std::vector<Eigen::Vector2i> cvMatToCoordinates(const cv::Mat& mat, int32_t blurSize, double pixelThreshold) {
    cv::Mat blured;
    cv::Mat imageGray;
    cv::Mat thresholded;
    std::vector<cv::Vec2i> locations;

    /*! - Grayscale, blur, and threshold image*/
    cv::cvtColor(mat, imageGray, cv::COLOR_BGR2GRAY);
    cv::blur(imageGray, blured, cv::Size(blurSize, blurSize));
    cv::threshold(blured, thresholded, pixelThreshold, 255, cv::THRESH_BINARY);

    /*! - Find all the non-zero pixels in the image*/
    cv::findNonZero(thresholded, locations);
    std::vector<Eigen::Vector2i> result;

    for (auto coord : locations) {
        auto location = Eigen::Vector2i(coord[0], coord[1]);
        result.push_back(location);
    }

    return result;
}

#endif
