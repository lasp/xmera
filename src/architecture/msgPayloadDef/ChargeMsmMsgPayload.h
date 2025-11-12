#ifndef CHARGE_MSM_MESSAGE_H
#define CHARGE_MSM_MESSAGE_H

#include <Eigen/Dense>

/*! @brief Structure used to define the MSM spacecraft sphere charge value message */
typedef struct
    //@cond DOXYGEN_IGNORE
    ChargeMsmMsgPayload
//@endcond
{
    Eigen::VectorXd q;  //!< [C], charge of each MSM sphere
} ChargeMsmMsgPayload;

#endif
