#ifndef LANDMARKIMMSG_H
#define LANDMARKIMMSG_H

/*! @brief Message that defines visibility of a landmark by the on-board camera providing the landmark pixel using
           the pinhole camera model
 */
typedef struct {
    int isVisible;  //!< [-] 1 when the landmark is visible by the camera, 0 otherwise.
    int pL[2];      //!< [-] landmark pixel as view from pinhole camera
} LandmarkMsgPayload;

#endif
