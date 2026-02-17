// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _IMAGE_PROC_CNN_H_
#define _IMAGE_PROC_CNN_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <Eigen/Dense>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavCirclesMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>

/*! @brief The CNN based center radius visual tracking module. */
class CenterRadiusCNN : public SysModel {
   public:
    CenterRadiusCNN();
    ~CenterRadiusCNN();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t currentSimNanos);

   public:
    std::string filename;                                //!< Filename for module to read an image directly
    Message<OpNavCirclesMsgPayload> opnavCirclesOutMsg;  //!< The name of the OpNavCirclesMsg output message

    ReadFunctor<CameraImageMsgPayload> imageInMsg;  //!< The name of the camera output message

    std::string pathToNetwork;  //!< Path to the trained CNN
    uint64_t sensorTimeTag;     //!< [ns] Current time tag for sensor out
    /* OpenCV specific arguments needed for HoughCircle finding*/
    int32_t saveImages;    //!< [-] 1 to save images to file for debugging
    double pixelNoise[3];  //!< [-] Pixel Noise for the estimate
    BSKLogger bskLogger;   //!< -- BSK Logging

   private:
    cv::dnn::Net positionNet2;  //!< Network for evaluation of centers
};

#endif
