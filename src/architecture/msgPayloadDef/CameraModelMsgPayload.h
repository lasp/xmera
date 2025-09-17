/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CAMERA_MODEL_MSG_H
#define CAMERA_MODEL_MSG_H

#define MAX_STRING_LENGTH 256
#define MAX_POLY_COEFF 10

/*! @brief Structure used to define the camera model*/

typedef struct {
    int cameraId;                        //!< [-]   ID of the camera that took the snapshot*/
    bool isOn;                           //!<  The camera is taking images at rendering rate if 1, 0 if not*/
    char parentName[MAX_STRING_LENGTH];  //!< [-] Name of the parent body to which the camera should be attached
    double fieldOfView[2];               //!< [rad]   Camera Field of View, edge-to-edge along camera y-axis */
    int resolution[2];  //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels*/
    uint64_t renderRate;  //!< [ns] Frame time interval at which to capture images in units of nanosecond */
    double cameraBodyFramePosition[3];  //!< [m] Camera position in body frame */
    double bodyToCameraMrp[3];  //!< [-] MRP defining the orientation of the camera frame relative to the body frame */
    double focalLength;         //!< [m] Camera focal length
    int gaussianPointSpreadFunction;   //!< Size of square Gaussian kernel to model point spread function, must be odd
    double exposureTime;               //!< [s] Exposure time for each image taken
    double readNoise;                  //!< [e-] Read noise standard deviation
    bool shotNoise;                    //!< [-] Model shot noise true or false
    double darkCurrent;                //!< [e-/s] Dark current variance value in electrons per second
    double systemGain;                 //!< Mapping from current to pixel intensity
    double gammaCorrection;            //!< Gamma correction factor for improved mid-tones
    double apertureRadius;             //!< [m] Aperture radius of lens
    double sensorWidth;                //!< [m] Width of sensor
    double sensorHeight;               //!< [m] Height of sensor
    double fullWellCapacity;           //!< [e-] Amount of charge that can be stored within an individual pixel
    double integrationWeightFactor;    //!< [-] Weight factor for integration over wavelength to obtain photo-electrons
    double redQuantumEfficiency[3];    //!< [-] Values of QE curve at specified wavelengths (red channel)
    double greenQuantumEfficiency[3];  //!< [-] Values of QE curve at specified wavelengths (green channel)
    double blueQuantumEfficiency[3];   //!< [-] Values of QE curve at specified wavelengths (blue channel)
    double horizontalVignetting[MAX_POLY_COEFF];  //!< [-] Polynomial coefficients to form the curve of vignetting
                                                  //!< (values between 0 and 1) as a function of horizontal distance
                                                  //!< from camera center (degrees)
    double verticalVignetting[MAX_POLY_COEFF];  //!< [-] Polynomial coefficients to form the curve of vignetting (values
                                                //!< between 0 and 1) as a function of vertical distance from camera
                                                //!< center (degrees)
    double distortion[MAX_POLY_COEFF];  //!< [-] Polynomial coefficients to form the curve of distortion (values between
                                        //!< 0 and 1) as a function of vertical distance from camera center (degrees)
    double transmission;  //!< [-] Transmission rate of the lens (value between 0 and 1) assumed constant over all
                          //!< wavelengths
} CameraModelMsgPayload;

#endif
