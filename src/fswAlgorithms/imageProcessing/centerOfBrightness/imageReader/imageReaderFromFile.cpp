#include "imageReaderFromFile.h"
#include "imageReaderUtilities.h"

/*! Get the time tag of the current image. File-based reader always returns 1 (no freshness tracking).
 * @param int32_t cameraId - the id of the camera of interest
 * @param int64_t previousImageTimeTag - the time tag of the latest image
 * @return int64_t the time tag of the new image (non-zero if present)
 */
int64_t ImageReaderFromFile::getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) { return 1; }

cv::Mat ImageReaderFromFile::readImageFromFile() {
    cv::Mat imageCV;
    if (!this->fileName.empty()) {
        imageCV = cv::imread(this->fileName, cv::IMREAD_COLOR);
    } else {
        throw std::invalid_argument("No filename provided for when requesting a file image read");
    }
    this->imageSize = Eigen::Vector2i(imageCV.cols, imageCV.rows);
    return imageCV;
}

/*! Read image and parse
 */
void ImageReaderFromFile::getImageAsArray(const Eigen::Vector2i& center,
                                          const Eigen::Vector2i& window,
                                          std::array<Eigen::Vector2i, kMaxWindowSize>& output) {
    cv::Mat imageCV = this->readImageFromFile();
    std::vector<Eigen::Vector2i> vector = cvMatToCoordinates(imageCV, this->blurSize, this->pixelThreshold);
    extractImageWindow(vector, center, window, output);
}

Eigen::Vector2i ImageReaderFromFile::getFullImageSize(int32_t cameraId) { return this->imageSize; };

void ImageReaderFromFile::setBlurSize(int blur) { this->blurSize = blur; }
int ImageReaderFromFile::getBlurSize() const { return this->blurSize; }
void ImageReaderFromFile::setPixelThreshold(double threshold) { this->pixelThreshold = threshold; }
double ImageReaderFromFile::getPixelThreshold() const { return this->pixelThreshold; }
void ImageReaderFromFile::setFileName(const std::string& fileNameInput) { this->fileName = fileNameInput; }
std::string ImageReaderFromFile::getFileName() const { return this->fileName; }
void ImageReaderFromFile::setSaveImages(bool save) { this->saveImages = save; }
bool ImageReaderFromFile::getSaveImages() const { return this->saveImages; }
void ImageReaderFromFile::setSaveDir(const std::string& directory) { this->saveDir = directory; }
std::string ImageReaderFromFile::getSaveDir() const { return this->saveDir; }
