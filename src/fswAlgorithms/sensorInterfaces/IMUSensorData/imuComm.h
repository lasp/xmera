#ifndef _IMU_COMM_H_
#define _IMU_COMM_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
#include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief module configuration message */
class ImuComm : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    double dcm_BP[9];                                 /*!< Row major platform 2 bdy DCM*/
    ReadFunctor<IMUSensorMsgPayload> imuComInMsg;     /*!< imu input message*/
    Message<IMUSensorBodyMsgPayload> imuSensorOutMsg; /*!< imu output message*/
    BSKLogger bskLogger = {};                         //!< BSK Logging
};

#endif
