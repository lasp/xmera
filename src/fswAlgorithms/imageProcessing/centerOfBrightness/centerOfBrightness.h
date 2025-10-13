/*
 ISC License

Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _IMAGE_PROC_COB_H_
#define _IMAGE_PROC_COB_H_

#include "architecture/messaging/messaging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/opencv.hpp"
#include <stdint.h>
#include <Eigen/Dense>

#include "architecture/msgPayloadDef/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOBMsgPayload.h"

#include <xmera/sys_model.h>
#include <xmera/bskLogging.h>

/*! @brief visual object tracking using center of brightness detection */
class CenterOfBrightness : public SysModel {
   public:
    CenterOfBrightness();
    ~CenterOfBrightness();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t currentSimNanos);

    void setWindowCenter(const Eigen::VectorXi &center);
    Eigen::VectorXi getWindowCenter() const;
    void setWindowSize(int32_t width, int32_t height);
    Eigen::VectorXi getWindowSize() const;
    void setRelativeBrightnessIncreaseThreshold(double increaseThreshold);
    double getRelativeBrightnessIncreaseThreshold() const;
    void setPixelThreshold(double PixelThreshold);
    double getPixelThreshold() const;
    void setFileName(const std::string &fileName);
    std::string getFileName() const;
    void setBlurSize(int32_t blur);
    int32_t getBlurSize() const;
    void setSaveImages(bool save);
    bool getSaveImages() const;
    void setSaveDir(const std::string &directory);
    std::string getSaveDir() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;

    Message<OpNavCOBMsgPayload> opnavCOBOutMsg;     //!< The name of the OpNav center of brightness output message
    ReadFunctor<CameraImageMsgPayload> imageInMsg;  //!< The name of the camera output message
    BSKLogger bskLogger;                            //!< -- BSK Logging

   private:
    cv::Mat readImage(CameraImageMsgPayload &imageBuffer, OpNavCOBMsgPayload &cobBuffer, uint64_t currentSimNanos);
    std::vector<cv::Vec2i> extractBrightPixels(cv::Mat image);
    std::pair<Eigen::Vector2d, double> computeWeightedCenterOfBrightness(std::vector<cv::Vec2i> nonZeroPixels);
    void computeWindow(cv::Mat const &image);
    void applyWindow(cv::Mat const &image) const;
    OpNavCOBMsgPayload findCob(const cv::Mat &imageCV, const CameraImageMsgPayload &imageBuffer);
    void updateBrightnessHistory(double brightness);

    uint64_t sensorTimeTag;                    //!< [ns] Current time tag for sensor out
    Eigen::VectorXi windowCenter{};            //!< [px] center of mask to be used for windowing
    int32_t windowWidth{};                     //!< [px] width of mask to be used for windowing
    int32_t windowHeight{};                    //!< [px] height of mask to be used for windowing
    Eigen::Vector2i windowPointTopLeft{};      //!< [px] top left point of window
    Eigen::Vector2i windowPointBottomRight{};  //!< [px] bottom right point of window
    bool validWindow = false;             //!< [px] true if window is set, false if center, height, or width equal 0
    Eigen::VectorXd brightnessHistory{};  //!< [-] brightness history to be used for rolling average
    double relativeBrightnessIncreaseThreshold{};  //!< [-] minimum relative brightness increase (if less, invalidated)
    double pixelThreshold{};  ////!< [-] minimum pixel brightness threshold used for detecting bright pixels
    std::string fileName{};   //!< Filename for module to read an image directly
    int32_t blurSize{};       //!< [px] Size of the blurring box in pixels
    bool saveImages{};        //!< [-] 1 to save images to file for debugging
    std::string saveDir{};    //!< The name of the directory to save images
    int32_t numberOfPointsBrightnessAverage{};  //!< [-] number of points to be used for rolling average of brightness
    /* OpenCV specific arguments needed for finding all non-zero pixels*/
    cv::Mat imageGray;  //!< [cv mat] Gray scale image for weighting
};

#endif
