#ifndef _IMAGE_LIMB_FIND_H_
#define _IMAGE_LIMB_FIND_H_

#include <architecture/messaging/messaging.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <Eigen/Dense>

#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavLimbMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>

/*! @brief visual limb finding module */
class LimbFinding : public SysModel {
   public:
    LimbFinding();
    ~LimbFinding();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t currentSimNanos);

   public:
    std::string filename;                           //!< Filename for module to read an image directly
    Message<OpNavLimbMsgPayload> opnavLimbOutMsg;   //!< The name of the Limb output message
    ReadFunctor<CameraImageMsgPayload> imageInMsg;  //!< The name of the camera output message
    std::string saveDir;                            //!< Directory to save images to

    uint64_t sensorTimeTag;  //!< [ns] Current time tag for sensor out
    /* OpenCV specific arguments needed for Limb finding*/
    int32_t blurrSize;        //!< [px] Size of the blurring box in pixels
    int32_t cannyThreshHigh;  //!< [px] Canny edge detection Threshold
    int32_t cannyThreshLow;   //!< [-] Second Threshold for Canny detection
    int32_t saveImages;       //!< [-] 1 to save images to file for debugging
    int32_t limbNumThresh;    //!< [-] Threshold for when a limb is detected

    BSKLogger bskLogger;  //!< -- BSK Logging
};

#endif
