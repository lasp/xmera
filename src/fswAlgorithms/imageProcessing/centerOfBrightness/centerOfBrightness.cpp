// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "centerOfBrightness.h"

/*! Module constructor */
CenterOfBrightness::CenterOfBrightness() = default;

/*! Module destructor */
CenterOfBrightness::~CenterOfBrightness() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::reset(uint64_t currentSimNanos) {
    if (!this->imageInMsg.isLinked()) {
        throw std::invalid_argument("CenterOfBrightness.imageInMsg wasn't connected.");
    }
    this->algorithm.reset();
}

/*! This module reads an OpNav image and extracts the weighted center of brightness. It performs a grayscale, a blur,
 * and a threshold on the image before summing the weighted pixel intensities in order to average them with the
 * total detected intensity. This provides the center of brightness measurement (as well as the total number of
 * bright pixels)
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::updateState(uint64_t currentSimNanos) {
    CameraImageMsgPayload imageBuffer{};
    OpNavCOBMsgPayload cobBuffer{};

    cv::Mat imageCV = this->readImage(imageBuffer, cobBuffer, currentSimNanos);
    if (imageCV.empty()) {
        this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, currentSimNanos);
        return;
    }

    CenterOfBrightnessResult result = this->algorithm.update(imageCV);

    cobBuffer.valid = result.valid;
    cobBuffer.centerOfBrightness[0] = result.centerOfBrightness[0];
    cobBuffer.centerOfBrightness[1] = result.centerOfBrightness[1];
    cobBuffer.pixelsFound = result.pixelsFound;
    cobBuffer.rollingAverageBrightness = result.rollingAverageBrightness;
    if (result.valid) {
        cobBuffer.timeTag = this->sensorTimeTag;
        cobBuffer.cameraID = imageBuffer.cameraID;
    }

    CenterOfBrightnessDiagnosticMsgPayload diagnosticBuffer{result.noPixelTrigger,
                                                            result.notExceedingBrightnessIncreaseTrigger};

    this->opnavCOBOutMsg.write(&cobBuffer, this->moduleID, currentSimNanos);
    this->centerOfBrightnessDiagnosticOutMsg.write(&diagnosticBuffer, this->moduleID, currentSimNanos);
}

/*! This method retrieves the image data.
@return OpenCV image or empty if no valid image is found.
@param imageBuffer Reference to the image payload buffer.
@param cobBuffer Reference to the COB output buffer.
@param currentSimNanos Current simulation time in nanoseconds.
 */
cv::Mat CenterOfBrightness::readImage(CameraImageMsgPayload& imageBuffer,
                                      OpNavCOBMsgPayload& cobBuffer,
                                      uint64_t currentSimNanos) {
    cv::Mat imageCV;

    /*! - Read in the image*/
    if (this->imageInMsg.isLinked()) {
        imageBuffer = this->imageInMsg();
        this->sensorTimeTag = this->imageInMsg.timeWritten();
    }
    /* Added for debugging purposes*/
    if (!this->fileName.empty()) {
        imageCV = cv::imread(this->fileName, cv::IMREAD_COLOR);
    } else if (imageBuffer.valid == 1 && imageBuffer.timeTag >= currentSimNanos) {
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer,
                                                (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
    }
    /*! - If no image is present, write zeros in message */
    else {
        return cv::Mat();
    }

    /*! - Save image to prescribed path if requested */
    std::string dirName;
    if (this->saveImages) {
        dirName = std::format("{}{}.png", this->saveDir, (double)currentSimNanos * NANO2SEC);
        if (!cv::imwrite(dirName, imageCV)) {
            std::cerr << "Warning: CenterOfBrightness wasn't able to save images." << std::endl;
        }
    }

    return imageCV;
}

/*! Delegating setters/getters for algorithm parameters */

void CenterOfBrightness::setWindowCenter(const Eigen::VectorXi& center) { this->algorithm.setWindowCenter(center); }

Eigen::VectorXi CenterOfBrightness::getWindowCenter() const { return this->algorithm.getWindowCenter(); }

void CenterOfBrightness::setWindowSize(const int32_t width, const int32_t height) {
    this->algorithm.setWindowSize(width, height);
}

Eigen::VectorXi CenterOfBrightness::getWindowSize() const { return this->algorithm.getWindowSize(); }

void CenterOfBrightness::setRelativeBrightnessIncreaseThreshold(double increaseThreshold) {
    this->algorithm.setRelativeBrightnessIncreaseThreshold(increaseThreshold);
}

double CenterOfBrightness::getRelativeBrightnessIncreaseThreshold() const {
    return this->algorithm.getRelativeBrightnessIncreaseThreshold();
}

void CenterOfBrightness::setPixelThreshold(double PixelThreshold) { this->algorithm.setPixelThreshold(PixelThreshold); }

double CenterOfBrightness::getPixelThreshold() const { return this->algorithm.getPixelThreshold(); }

void CenterOfBrightness::setBlurSize(int32_t blur) { this->algorithm.setBlurSize(blur); }

int32_t CenterOfBrightness::getBlurSize() const { return this->algorithm.getBlurSize(); }

void CenterOfBrightness::setNumberOfPointsBrightnessAverage(int32_t rollingAverage) {
    this->algorithm.setNumberOfPointsBrightnessAverage(rollingAverage);
}

int32_t CenterOfBrightness::getNumberOfPointsBrightnessAverage() const {
    return this->algorithm.getNumberOfPointsBrightnessAverage();
}

/*! Adapter-only setters/getters */

void CenterOfBrightness::setFileName(const std::string& fileName) { this->fileName = fileName; }

std::string CenterOfBrightness::getFileName() const { return this->fileName; }

void CenterOfBrightness::setSaveImages(bool save) { this->saveImages = save; }

bool CenterOfBrightness::getSaveImages() const { return this->saveImages; }

void CenterOfBrightness::setSaveDir(const std::string& directory) { this->saveDir = directory; }

std::string CenterOfBrightness::getSaveDir() const { return this->saveDir; }
