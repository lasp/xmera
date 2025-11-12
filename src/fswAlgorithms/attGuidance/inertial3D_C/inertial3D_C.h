#ifndef INERTIAL3D_C
#define INERTIAL3D_C

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3D_C : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    double sigma_R0N[3];                     //!<  MRP from inertial frame N to corrected reference frame R
    Message<AttRefMsgPayload> attRefOutMsg;  //!< reference attitude output message
    BSKLogger bskLogger = {};                //!< BSK Logging
};

#endif
