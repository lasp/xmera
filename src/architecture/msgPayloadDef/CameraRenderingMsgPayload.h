// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CAMERA_RENDERING
#define CAMERA_RENDERING

#define MAX_STRING_LENGTH 256

/*! @brief Message containing rendering information for a specific camera*/

typedef struct {
    int cameraId;
    double cosmicRayStdDeviation;
    bool strayLightEnabled;               //!< [-] Master toggle for the stray-light / lens-flare pass (default false)
    double strayLightCoreSize;            //!< [-] Scale factor for the sun core blob size [0.1, 1] (default 1.0)
    double strayLightGhostSize;           //!< [-] Global scale factor for the ghost sizes [0.1, 1.25] (default 1.0)
    double strayLightGhostTransmittance;  //!< [-] Global brightness scale for the ghosts [0.5, 1.5] (default 3.0)
    double strayLightGhost1RelativeSize;  //!< [-] Per-ghost size scale for the first (closest) ghosts [0.25, 1]
                                          //!< (default 1.0)
    double strayLightGhost2RelativeSize;  //!< [-] Per-ghost size scale for the second ghosts [0.25, 1] (default 1.0)
    double strayLightGhost3RelativeSize;  //!< [-] Per-ghost size scale for the third ghosts [0.25, 1] (default 1.0)
    double
        strayLightGhost4RelativeSize;  //!< [-] Per-ghost size scale for the fourth (orb) ghosts [0.25, 1] (default 1.0)
    double strayLightGhostBrightnessSizeExponent;  //!< [-] Couples ghost brightness to inverse size (default 2.0)
    double strayLightCoronaFalloffExponent;  //!< [-] Falloff exponent of the wide corona aureole (higher = tighter)
                                             //!< [0.5, 2] (default 1.2)
    double strayLightCoronaIntensity;    //!< [-] Brightness amplitude of the wide corona, relative to the core [0, 1]
                                         //!< (default 0.02)
    double strayLightBaffleShieldAngle;  //!< [deg] Extra angle beyond the FoV half-angle out to which the off-axis sun
                                         //!< still casts stray light (default 0.0 = only when the sun is in frame)
    double strayLightIntensity;  //!< [-] Overall flare brightness as a fraction of direct solar radiance; scales the
                                 //!< whole flare (default 1.0)
    double strayLightNumRays;    //!< [-] Number of evenly-spaced symmetric rays from the sun (even -> mirror-symmetric)
                                 //!< [0, 15] (default 6.0)
    double strayLightRaySharpness;  //!< [-] Angular sharpness of the rays (higher = narrower) [0, 30] (default 24.0)
    double strayLightRayWeight;  //!< [-] Strength of the symmetric rays relative to the random streaks [0, 1] (default
                                 //!< 0.8)
    bool starField;
    char rendering[MAX_STRING_LENGTH];
    bool smear;
    double wavelengths[3];  //!< [nm] wavelength sample points
} CameraRenderingMsgPayload;

#endif  // CAMERA_RENDERING
