// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#include <architecture/messaging/messaging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <Eigen/Dense>

#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/PairedKeyPointsMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.h>

/*! @brief feature tracking module */
class OpticalFlow : public SysModel {
   public:
    OpticalFlow();
    ~OpticalFlow();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t currentSimNanos);
    void makeMask(cv::Mat const& inputBWImage, cv::Mat& mask) const;

   private:
    cv::Mat firstImage;
    cv::Mat secondImage;
    bool firstImagePresent = false;
    bool secondImagePresent = false;
    double firstSpacecraftAttitude[3];
    double firstTargetEphemAttitude[3];
    uint64_t firstTimeTag = 0;
    std::vector<cv::Vec2f> secondFeatures;
    std::vector<cv::Vec2f> firstFeatures;

   public:
    std::string directoryName = "";           //!< Directory name for module to read an image directly
    std::string imageFileExtension = ".png";  //!< Directory name for module to read an image directly

    Message<PairedKeyPointsMsgPayload> keyPointsMsg;  //!< The name of the output message containing key points
    ReadFunctor<CameraImageMsgPayload> imageInMsg;    //!< The name of the camera output message containing images
    ReadFunctor<NavAttMsgPayload> attitudeMsg;        //!< The name of the input attitude information
    ReadFunctor<EphemerisMsgPayload> ephemerisMsg;    //!< The name of the input central target ephemeris data
    uint64_t sensorTimeTag;                           //!< [ns] Current time tag for sensor out

    double minTimeBetweenPairs = 1;    //!< [s] Minimum time between pairs of images
    bool slidingWindowImages = false;  //!< [bool] Minimum time between pairs of images

    /*! OpenCV specific arguments needed for Shi-Tomasi "good features to track" */
    int32_t maxNumberFeatures = 100;
    double qualityLevel = 0.3;
    int32_t minumumFeatureDistance = 5;
    int32_t blockSize = 7;

    /*! OpenCV specific arguments needed for OpticalFlow */
    int32_t criteriaMaxCount = 10;
    double criteriaEpsilon = 0.03;
    int32_t flowSearchSize = 10;
    int32_t flowMaxLevel = 2;

    /*! OpenCV specific arguments needed for masking */
    int32_t thresholdMask = 20;
    int32_t limbMask = 20;

    BSKLogger bskLogger;
};

#endif
