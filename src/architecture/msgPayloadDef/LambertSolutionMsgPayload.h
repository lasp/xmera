#ifndef LAMBERT_SOLUTION_MESSAGE_H
#define LAMBERT_SOLUTION_MESSAGE_H

/*! @brief Structure used to define the output of the Lambert problem solution */
typedef struct {
    double v1[3];      //!< [m/s] velocity solution at t1
    double v2[3];      //!< [m/s] velocity solution at t2
    int valid;         //!< [-] valid solution if 1, not if 0
    double v1Sol2[3];  //!< [m/s] second velocity solution at t1 (for multi-revolution solutions)
    double v2Sol2[3];  //!< [m/s] second velocity solution at t2 (for multi-revolution solutions)
    int validSol2;     //!< [-] valid second solution if 1, not if 0
} LambertSolutionMsgPayload;

#endif
