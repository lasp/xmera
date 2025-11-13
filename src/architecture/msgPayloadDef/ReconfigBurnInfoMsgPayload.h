#ifndef RECONFIG_BURN_INFO_H
#define RECONFIG_BURN_INFO_H

//! @brief Container for the orbit reconfiguration burn information.
typedef struct {
    int flag;  //!< 0:not scheduled yet, 1:not burned yet, 2:already burned, 3:skipped (combined with another burn)
    double t;  //!< [s] simulation time of the scheduled burn since module reset
    double thrustOnTime;  //!< thrust on duration time [s]
    double sigma_RN[3];   //!< target attitude
} ReconfigBurnInfoMsgPayload;

#endif
