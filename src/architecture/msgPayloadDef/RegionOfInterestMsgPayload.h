#ifndef REGION_OF_INTEREST_H
#define REGION_OF_INTEREST_H

/*! @brief Regions of interest */
typedef struct {
    double timeTag;           //!< [sec] time tag of the image which provided the region(s)
    int centerX;              //!< [pix] center pixel coordinate x
    int centerY;              //!< [pix] center pixel coordinate y
    int width;                //!< [pix] region width
    int height;               //!< [pix] region height
    int numberOfPixels;       //!< [-] number of pixels in the region
    int centerOfBrightnessX;  //!< [pix] center brightness pixel coordinate x
    int centerOfBrightnessY;  //!< [pix] center brightness pixel coordinate y
} RegionOfInterestMsgPayload;

#endif
