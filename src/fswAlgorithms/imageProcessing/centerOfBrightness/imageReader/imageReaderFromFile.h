#ifndef _IMAGE_READER_FROM_FILE_H_
#define _IMAGE_READER_FROM_FILE_H_

#include "imageReaderInterface.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>

class ImageReaderFromFile : public ImageReaderInterface {
   public:
    ImageReaderFromFile() = default;
    ~ImageReaderFromFile() override = default;
    Eigen::Vector2i getFullImageSize(int32_t cameraId) final;
    void getImageAsArray(const Eigen::Vector2i& center,
                         const Eigen::Vector2i& windowSize,
                         std::array<Eigen::Vector2i, kMaxWindowSize>& output) final;
    int64_t getCurrentImageTimeTag(int32_t cameraId, int64_t previousImageTimeTag) final;

    void setBlurSize(int blur);
    int getBlurSize() const;
    void setPixelThreshold(double threshold);
    double getPixelThreshold() const;
    void setFileName(const std::string& fileName);
    std::string getFileName() const;
    void setSaveImages(bool save);
    bool getSaveImages() const;
    void setSaveDir(const std::string& directory);
    std::string getSaveDir() const;

   private:
    cv::Mat readImageFromFile();

    std::string fileName{};       //!< Filename to read an image directly
    Eigen::Vector2i imageSize{};  //!< [ns] Current time tag of image
    double pixelThreshold{};      ////!< [-] minimum pixel brightness threshold used for detecting bright pixels
    int32_t blurSize{};           //!< [px] Size of the blurring box in pixels
    bool saveImages{};            //!< [-] flag to save images on each getImageAsArray call
    std::string saveDir{};        //!< [-] path to save the image to
};

#endif
