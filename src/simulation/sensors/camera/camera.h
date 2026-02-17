// SPDX-License-Identifier: ISC
// Copyright (c) 2018, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string_view>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
#include <architecture/msgPayloadDef/CameraModelMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>

/*! @brief visual camera class */
class Camera : public SysModel {
   public:
    Camera();
    ~Camera();

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;
    void hsvAdjust(const cv::Mat&, cv::Mat& mDst);
    void bgrAdjustPercent(const cv::Mat&, cv::Mat& mDst);
    void addGaussianNoise(const cv::Mat&, cv::Mat& mDst, double, double);
    void addSaltPepper(const cv::Mat&, cv::Mat& mDst, float, float);
    void addCosmicRay(const cv::Mat&, cv::Mat& mDst, float, double, int);
    void addCosmicRayBurst(const cv::Mat&, cv::Mat& mDst, double);
    void applyFilters(cv::Mat& mSource, cv::Mat& mDst);

    void setParentName(const std::string& cameraParentName);
    std::string getParentName() const;
    void setCameraOn();
    void setCameraOff();
    bool isCameraOn() const;
    void setCameraId(int cameraId);
    int getCameraId() const;
    void setResolution(const Eigen::Vector2i& cameraResolution);
    Eigen::Vector2i getResolution() const;
    void setImageCadence(const uint64_t& cameraRenderRate);
    uint64_t getImageCadence() const;
    void setFieldOfView(const Eigen::Vector2d& fov);
    Eigen::Vector2d getFieldOfView() const;
    void setCameraBodyFramePosition(const Eigen::Vector3d& cameraPosition_B);
    Eigen::Vector3d getCameraBodyFramePosition() const;
    void setBodyToCameraMrp(const Eigen::Vector3d& cameraMrp_CB);
    Eigen::Vector3d getBodyToCameraMrp() const;
    void setFocalLength(double cameraFocalLength);
    double getFocalLength() const;
    void setGaussianPointSpreadFunction(int cameraGaussianPointSpreadFunction);
    int getGaussianPointSpreadFunction() const;
    void setReadNoise(double cameraReadNoise);
    double getReadNoise() const;
    void setShotNoise(bool cameraShotNoise);
    bool getShotNoise() const;
    void setDarkCurrent(double cameraDarkCurrent);
    double getDarkCurrent() const;
    void setSystemGain(double cameraGain);
    double getSystemGain() const;
    void setExposureTime(double exposureTime);
    double getExposureTime() const;
    void setGammaCorrection(double gammaCorrection);
    double getGammaCorrection() const;
    void setApertureRadius(double apertureRadiusValue);
    double getApertureRadius() const;
    void setSensorWidth(double sensorWidthValue);
    double getSensorWidth() const;
    void setSensorHeight(double sensorHeightValue);
    double getSensorHeight() const;
    void setIntegrationWeightFactor(double integrationWeightFactorValue);
    double getIntegrationWeightFactor() const;
    void setFullWellCapacity(double fullWellCapacityValue);
    double getFullWellCapacity() const;
    void setRedQuantumEfficiency(const Eigen::Vector3d& redQE);
    Eigen::Vector3d getRedQuantumEfficiency() const;
    void setGreenQuantumEfficiency(const Eigen::Vector3d& greenQE);
    Eigen::Vector3d getGreenQuantumEfficiency() const;
    void setBlueQuantumEfficiency(const Eigen::Vector3d& blueQE);
    Eigen::Vector3d getBlueQuantumEfficiency() const;
    void setHorizontalVignetting(const Eigen::VectorXd& horizontalVignettingCoeffs);
    Eigen::VectorXd getHorizontalVignetting() const;
    void setVerticalVignetting(const Eigen::VectorXd& verticalVignettingCoeffs);
    Eigen::VectorXd getVerticalVignetting() const;
    void setDistortion(const Eigen::VectorXd& distortionCoeffs);
    Eigen::VectorXd getDistortion() const;
    void setTransmission(double transmissionValue);
    double getTransmission() const;

   private:
    std::string parentSpacecraftName{};  //!< [-] Name of the parent body to which the camera should be attached
    bool cameraIsImaging{};              //!< [-] Is the camera currently taking images
    int cameraIdentification{1};         //!< [-] Camera identification
    Eigen::Vector2i resolution{
        512,
        512};  //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels
    uint64_t imageCadence{static_cast<uint64_t>(
        60 * 1E9)};  //!< [ns] Frame time interval at which to capture images in units of nanosecond
    Eigen::Vector2d cameraFieldOfView{0.7, 0.7};  //!< [r] camera y-axis field of view edge-to-edge
    Eigen::Vector3d cameraBodyFramePosition{};    //!< [m] Camera position in body frame
    Eigen::Vector3d
        bodyToCameraMrp{};  //!< [-] MRP defining the orientation of the camera frame relative to the body frame
    double focalLength{};   //!< [m] Camera focal length
    int gaussianPointSpreadFunction{};  //!< Size of square Gaussian kernel to model point spread function, must be odd
    double readNoise{};                 //!< [e-] Read noise standard deviation
    bool shotNoise{};                   //!< [-] Model shot noise true or false
    double darkCurrent{};               //!< [e-/s] Dark current variance value in electrons per second
    double systemGain{};                //!< Mapping from current to pixel intensity
    double exposureTime{1};             //!< [s] Time with open shutter
    double gammaCorrection{1};          //!< Gamma correction factor
    double apertureRadius{};            //!< [m] Aperture radius of lens
    double sensorWidth{};               //!< [m] Width of sensor
    double sensorHeight{};              //!< [m] Height of sensor
    double fullWellCapacity{};          //!< [e-] Amount of charge that can be stored within an individual pixel
    double integrationWeightFactor{1};  //!< [-] Weight factor for integration over wavelength to obtain photo-electrons
    Eigen::Vector3d redQuantumEfficiency{};    //!< [-] Values of QE curve at specified wavelengths (red channel)
    Eigen::Vector3d greenQuantumEfficiency{};  //!< [-] Values of QE curve at specified wavelengths (green channel)
    Eigen::Vector3d blueQuantumEfficiency{};   //!< [-] Values of QE curve at specified wavelengths (blue channel)
    Eigen::VectorXd
        horizontalVignetting{};  //!< [-] Polynomial coefficients to form the curve of vignetting (values between 0 and
                                 //!< 1) as a function of horizontal distance from camera center (degrees)
    Eigen::VectorXd
        verticalVignetting{};  //!< [-] Polynomial coefficients to form the curve of vignetting (values between 0 and 1)
                               //!< as a function of vertical distance from camera center (degrees)
    Eigen::VectorXd distortion{};  //!< [-] Polynomial coefficients to form the curve of distortion (values between 0
                                   //!< and 1) as a function of vertical distance from camera center (degrees)
    double transmission{};  //!< [-] Transmission rate of the lens (value between 0 and 1) assumed constant over all
                            //!< wavelengths

   public:
    std::string filename{};                              //!< Filename for module to read an image directly
    ReadFunctor<CameraImageMsgPayload> imageInMsg;       //!< camera image input message
    Message<CameraImageMsgPayload> imageOutMsg;          //!< camera image output message
    Message<CameraConfigMsgPayload> cameraConfigOutMsg;  //!< The name of the CameraConfigMsg output message
    Message<CameraModelMsgPayload> cameraModelOutMsg;    //!< The name of the CameraModelMsg output message
    std::string saveDir{};                               //!< The name of the directory to save images
    uint64_t sensorTimeTag{};                            //!< [ns] Current time tag for sensor out
    int32_t saveImages{};                                //!< [-] 1 to save images to file for debugging

    /*! Camera parameters */
    char parentName[MAX_STRING_LENGTH]{};  //!< [-] Name of the parent body to which the camera should be attached
    int cameraIsOn{};                      //!< [-] Is the camera currently taking images
    int cameraId{1};                       //!< [-] Is the camera currently taking images
    uint64_t renderRate{static_cast<uint64_t>(
        60 * 1E9)};           //!< [ns] Frame time interval at which to capture images in units of nanosecond
    double fieldOfView{0.7};  //!< [r] camera y-axis field of view edge-to-edge
    double cameraPos_B[3]{};  //!< [m] Camera position in body frame
    double sigma_CB[3]{};     //!< [-] MRP defining the orientation of the camera frame relative to the body frame
    char skyBox[MAX_STRING_LENGTH]{"black"};  //!< [-] name of skyboz in use
    int postProcessingOn{};    //!< Enable post-processing of camera image. Value of 0 (protobuffer default) to use viz
                               //!< default which is off, -1 for false, 1 for true
    double ppFocusDistance{};  //!< Distance to the point of focus, minimum value of 0.1, Value of 0 to turn off this
                               //!< parameter entirely.
    double ppAperture{};       //!<  Ratio of the aperture (known as f-stop or f-number). The smaller the value is, the
                          //!<  shallower the depth of field is. Valid Setting Range: 0.05 to 32. Value of 0 to turn off
                          //!<  this parameter entirely.
    double
        ppFocalLength{};  //!< [m] Valid setting range: 0.001m to 0.3m. Value of 0 to turn off this parameter entirely.
    int ppMaxBlurSize{};  //!< Convolution kernel size of the bokeh filter, which determines the maximum radius of
                          //!< bokeh. It also affects the performance (the larger the kernel is, the longer the GPU time
                          //!< is required). Depth textures Value of 1 for Small, 2 for Medium, 3 for Large, 4 for Extra
                          //!< Large. Value of 0 to turn off this parameter entirely.

    /*! Noise paramters */
    double gaussian{};                             //!< Gaussian noise level
    double darkCurrentIntensity{};                 //!< Dark current intensity
    double saltPepper{};                           //!< Stuck and Dark pixels probability
    double cosmicRays{};                           //!< Random cosmic rays (number)
    double blurParam{};                            //!< Blur over image in pixels
    Eigen::Vector3d hsv{Eigen::Vector3d::Zero()};  //!< (double) HSV color correction, H (-pi/pi) hue shift, S and V are
                                                   //!< percent multipliers
    Eigen::Vector3d bgrPercent{Eigen::Vector3d::Zero()};  //!< (int) BGR color correction values as percent

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    uint64_t localcurrentSimNanos{};
    void* pointImageOut{nullptr};  //!< void pointer for image memory passing
};

/* @} */
#endif
