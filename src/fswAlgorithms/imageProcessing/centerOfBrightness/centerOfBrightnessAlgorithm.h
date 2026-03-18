// SPDX-License-Identifier: ISC
// Copyright (c) 2018, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _IMAGE_PROC_COB_ALGORITHM_H_
#define _IMAGE_PROC_COB_ALGORITHM_H_

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <stdint.h>

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
 * @brief Center of brightness detection algorithm
 *
 * Processes an image to find the weighted center of brightness. Performs
 * grayscale conversion, blurring, thresholding, and optional windowing
 * before computing the intensity-weighted centroid of bright pixels.
 */
class CenterOfBrightnessAlgorithm {
   public:
    CenterOfBrightnessAlgorithm();
    ~CenterOfBrightnessAlgorithm();

    CenterOfBrightnessResult update(const cv::Mat& image);
    void reset();

    void setWindowCenter(const Eigen::VectorXi& center);
    Eigen::VectorXi getWindowCenter() const;
    void setWindowSize(int32_t width, int32_t height);
    Eigen::VectorXi getWindowSize() const;
    void setRelativeBrightnessIncreaseThreshold(double increaseThreshold);
    double getRelativeBrightnessIncreaseThreshold() const;
    void setPixelThreshold(double pixelThreshold);
    double getPixelThreshold() const;
    void setBlurSize(int32_t blur);
    int32_t getBlurSize() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;

   private:
    std::vector<cv::Vec2i> extractBrightPixels(cv::Mat image);
    std::pair<Eigen::Vector2d, double> computeWeightedCenterOfBrightness(std::vector<cv::Vec2i> nonZeroPixels);
    void computeWindow(cv::Mat const& image);
    void applyWindow(cv::Mat const& image) const;
    CenterOfBrightnessResult findCob(const cv::Mat& imageCV);
    void updateBrightnessHistory(double brightness);

    Eigen::VectorXi windowCenter{};            //!< [px] center of mask to be used for windowing
    int32_t windowWidth{};                     //!< [px] width of mask to be used for windowing
    int32_t windowHeight{};                    //!< [px] height of mask to be used for windowing
    Eigen::Vector2i windowPointTopLeft{};      //!< [px] top left point of window
    Eigen::Vector2i windowPointBottomRight{};  //!< [px] bottom right point of window
    bool validWindow = false;             //!< [px] true if window is set, false if center, height, or width equal 0
    Eigen::VectorXd brightnessHistory{};  //!< [-] brightness history to be used for rolling average
    double relativeBrightnessIncreaseThreshold{};  //!< [-] minimum relative brightness increase (if less, invalidated)
    double pixelThreshold{};  //!< [-] minimum pixel brightness threshold used for detecting bright pixels
    int32_t blurSize{};       //!< [px] Size of the blurring box in pixels
    int32_t numberOfPointsBrightnessAverage{};  //!< [-] number of points to be used for rolling average of brightness
    cv::Mat imageGray;                          //!< [cv mat] Gray scale image for weighting
};

#endif
