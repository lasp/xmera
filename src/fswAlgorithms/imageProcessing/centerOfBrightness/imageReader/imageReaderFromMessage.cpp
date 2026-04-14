#include "imageReaderFromMessage.h"
#include "imageReaderUtilities.h"

/*! Get the time tag of the current image if it is newer than the previous one
 * @param int32_t cameraId - the id of the camera of interest
 * @param int64_t previousImageTimeTag - the time tag of the latest image
 * @return int64_t the time tag of the new image (non-zero if present)
 */
int64_t ImageReaderFromMessage::getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) {
    /*! - Read in the image*/
    this->imagePayload = this->imageInMsg();
    this->imageTimeTag = this->imagePayload.timeTag;

    /*! - If an image is present and fresh, return the time tag of this new image. If no new image is present, return 0
     */
    if (this->imagePayload.valid == 1 && this->imageTimeTag > previousImageTimeTag) {
        return this->imageTimeTag;
    }
    return 0;
}

cv::Mat ImageReaderFromMessage::readImageFromMessage() {
    this->imagePayload = this->imageInMsg();
    cv::Mat imageCV{};
    if (this->imagePayload.valid == 1) {
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer(
            (char*)this->imagePayload.imagePointer,
            (char*)this->imagePayload.imagePointer + this->imagePayload.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
    }
    if (!imageCV.empty()) {
        this->imageSize = Eigen::Vector2i(imageCV.cols, imageCV.rows);
    }
    return imageCV;
}

/*! Read image and parse
 */
void ImageReaderFromMessage::getImageAsArray(const Eigen::Vector2i& center,
                                             const Eigen::Vector2i& window,
                                             std::array<Eigen::Vector2i, kMaxWindowSize>& output) {
    cv::Mat imageCV = this->readImageFromMessage();
    if (imageCV.empty()) {
        output.fill(Eigen::Vector2i::Zero());
        return;
    }
    auto vector = cvMatToCoordinates(imageCV, this->blurSize, this->pixelThreshold);
    extractImageWindow(vector, center, window, output);
}

Eigen::Vector2i ImageReaderFromMessage::getFullImageSize(int32_t cameraId) { return this->imageSize; };
void ImageReaderFromMessage::setBlurSize(int blur) { this->blurSize = blur; }
int ImageReaderFromMessage::getBlurSize() const { return this->blurSize; }
void ImageReaderFromMessage::setPixelThreshold(double threshold) { this->pixelThreshold = threshold; }
double ImageReaderFromMessage::getPixelThreshold() const { return this->pixelThreshold; }
