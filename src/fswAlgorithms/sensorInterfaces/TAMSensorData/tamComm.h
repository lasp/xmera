#ifndef _TAM_COMM_H_
#define _TAM_COMM_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
#include <architecture/msgPayloadDef/TAMSensorMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! module configuration message definition */
class TamComm : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    double dcm_BS[9];                            //!< [T] Row - Sensor to Body DCM
    ReadFunctor<TAMSensorMsgPayload> tamInMsg;   //!< [-] TAM interface input message
    Message<TAMSensorBodyMsgPayload> tamOutMsg;  //!< [-] TAM interface output message

    TAMSensorBodyMsgPayload tamLocalOutput;  //!< [-] buffer of TAM output data structure
    BSKLogger bskLogger{};                   //!< BSK Logging
};

#endif
