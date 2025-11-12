#ifndef HILL_NAV_MESSAGE_H
#define HILL_NAV_MESSAGE_H

/*! @brief Structure used to define the output definition for attitude reference generation */
typedef struct {
    double r_DC_H[3];  //!< [m]  Relative position of the deputy to the chief in Hill-frame components
    double v_DC_H[3];  //!< [m/s]  Relative velocity of the deputy to the chief in Hill-frame components
} HillRelStateMsgPayload;

#endif
