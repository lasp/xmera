#ifndef _INERTIAL3D_SPIN_
#define _INERTIAL3D_SPIN_

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>

/*!@brief module configuration structure definition.
 */
class Inertial3DSpin : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void computeReference_inertial3DSpin(double omega_R0N_N[3], double domega_R0N_N[3], double dt);
    /* declare module private variables */
    double sigma_RN[3];     /*!< MPR of reference frame relative to inertial N frame */
    double omega_RR0_R0[3]; /*!< [r/s] constant angular velocity spin vector of the spinning R frame relative to the
                               input frame R0 */
    uint64_t priorTime;     /*!< [ns] last time the guidance module is called */
    /* declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;     //!< reference attitude output message
    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< (optional) reference attitude input message

    AttRefMsgPayload attRefOutBuffer;  //!< [-] structure for the output data
    BSKLogger bskLogger = {};          //!< BSK Logging
};

#endif
