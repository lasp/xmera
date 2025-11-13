/*
    Hough Circle Finder

    Note:   This module takes an image and writes out the circles that are found in the image by OpenCV's HoughCricle
   Transform. Author: Thibaud Teil Date:   February 13, 2019

 */

/* modify the path to reflect the new module names */
#include "houghCircles.h"

/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
HoughCircles::HoughCircles() {
    this->filename = "";
    this->saveImages = 0;
    this->noiseSF = 4;
    this->blurrSize = 5;
    this->dpValue = 1;
    this->expectedCircles = MAX_CIRCLE_NUM;
    this->saveDir = "";
    this->cannyThresh = 200;
    this->voteThresh = 20;
    this->houghMinDist = 50;
    this->houghMinRadius = 0;
    this->houghMaxRadius = 0;  // Maximum circle radius. If <= 0, uses the maximum image dimension. If < 0, returns
                               // centers without finding the radius
}

/*! This is the destructor */
HoughCircles::~HoughCircles() { return; }

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between
 function calls are reset to their default values.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HoughCircles::reset(uint64_t currentSimNanos) {
    // check that the required message has not been connected
    if (!this->imageInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HoughCircles.imageInMsg wasn't connected.");
    }
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle
 Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HoughCircles::updateState(uint64_t currentSimNanos) {
    std::string dirName;
    CameraImageMsgPayload imageBuffer{};
    OpNavCirclesMsgPayload circleBuffer{};

    cv::Mat imageCV, blurred;
    int circlesFound = 0;
    if (this->saveDir != "") {
        dirName = this->saveDir + std::to_string(currentSimNanos * 1E-9) + ".jpg";
    } else {
        dirName = "./" + std::to_string(currentSimNanos * 1E-9) + ".jpg";
    }
    /*! - Read in the bitmap*/
    if (this->imageInMsg.isLinked()) {
        imageBuffer = this->imageInMsg();
        this->sensorTimeTag = this->imageInMsg.timeWritten();
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()) {
        imageCV = cv::imread(this->filename, cv::IMREAD_COLOR);
    } else if (imageBuffer.valid == 1 && imageBuffer.timeTag >= currentSimNanos) {
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer,
                                                (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
        if (this->saveImages == 1) {
            if (!cv::imwrite(this->saveDir, imageCV)) {
                bskLogger.bskLog(BSK_WARNING, "houghCircles: wasn't able to save images.");
            }
        }
    } else {
        /*! - If no image is present, write zeros in message */
        this->opnavCirclesOutMsg.write(&circleBuffer, this->moduleID, currentSimNanos);
        return;
    }

    cv::cvtColor(imageCV, imageCV, cv::COLOR_BGR2GRAY);
    cv::threshold(imageCV, imageCV, 15, 255, cv::THRESH_BINARY_INV);
    cv::blur(imageCV, blurred, cv::Size(this->blurrSize, this->blurrSize));

    std::vector<cv::Vec4f> circles;
    /*! - Apply the Hough Transform to find the circles*/
    cv::HoughCircles(blurred,
                     circles,
                     cv::HOUGH_GRADIENT,
                     this->dpValue,
                     this->houghMinDist,
                     this->cannyThresh,
                     this->voteThresh,
                     this->houghMinRadius,
                     this->houghMaxRadius);

    circleBuffer.timeTag = this->sensorTimeTag;
    circleBuffer.cameraID = imageBuffer.cameraID;
    for (int i = 0; i < this->expectedCircles && i < (int)circles.size(); i++) {
        circleBuffer.circlesCenters[2 * i] = circles[i][0];
        circleBuffer.circlesCenters[2 * i + 1] = circles[i][1];
        circleBuffer.circlesRadii[i] = circles[i][2];
        for (int j = 0; j < 3; j++) {
            circleBuffer.uncertainty[j + 3 * j] = this->noiseSF * circles[i][3] / this->voteThresh;
        }
        circlesFound += 1;
    }
    /*!- If no circles are found do not validate the image as a measurement */
    if (circlesFound > 0) {
        circleBuffer.valid = 1;
        circleBuffer.planetIds[0] = 2;
    }

    this->opnavCirclesOutMsg.write(&circleBuffer, this->moduleID, currentSimNanos);

    //    free(imageBuffer.imagePointer);
    return;
}
