#ifndef CENTER_OF_BRIGHTNESS_DIAGNOSTIC_MESSAGE_H
#define CENTER_OF_BRIGHTNESS_DIAGNOSTIC_MESSAGE_H

/*! @brief Structure used to define  */
typedef struct {
    bool noPixelTrigger;                         // true if no pixels found in the image
    bool notExceedingBrightnessIncreaseTrigger;  // true if the brightness increase does not exceed the specified
                                                 // threshold: relativeBrightnessIncreaseThreshold
} CenterOfBrightnessDiagnosticMsgPayload;

#endif
