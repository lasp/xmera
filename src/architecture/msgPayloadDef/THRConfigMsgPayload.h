#ifndef FSW_THR_CONFIG_MESSAGE_H
#define FSW_THR_CONFIG_MESSAGE_H

/*! @brief Single Thruster configuration message */
typedef struct {
    double rThrust_B[3];     //!< [m] Location of the thruster in the spacecraft
    double tHatThrust_B[3];  //!< [-] Unit vector of the thrust direction
    double maxThrust;        //!< [N] Max thrust
} THRConfigMsgPayload;

#endif
