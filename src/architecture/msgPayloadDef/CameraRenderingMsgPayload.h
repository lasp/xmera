#ifndef CAMERA_RENDERING
#define CAMERA_RENDERING

#define MAX_STRING_LENGTH 256

/*! @brief Message containing rendering information for a specific camera*/

typedef struct {
    int cameraId;
    double cosmicRayStdDeviation;
    double strayLight;
    bool starField;
    char rendering[MAX_STRING_LENGTH];
    bool smear;
    double wavelengths[3];  //!< [nm] wavelength sample points
} CameraRenderingMsgPayload;

#endif  // CAMERA_RENDERING
