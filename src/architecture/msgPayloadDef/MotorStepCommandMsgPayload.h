#ifndef motorStepCommandSimMsg_h
#define motorStepCommandSimMsg_h

/*! @brief Structure containing number of commanded stepper motor steps */
typedef struct {
    int stepsCommanded;  //!< Number of commanded stepper motor steps
} MotorStepCommandMsgPayload;

#endif /* motorStepCommandSimMsg_h */
