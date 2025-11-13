#ifndef RW_MOTOR_VOLTAGE_C_H
#define RW_MOTOR_VOLTAGE_C_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*!@brief module configuration message
 */

class RwMotorVoltage_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    /* declare module private variables */
    double VMin;                   /*!< [V]    minimum voltage below which the torque is zero */
    double VMax;                   /*!< [V]    maximum output voltage */
    double K;                      /*!< [V/Nm] torque tracking gain for closed loop control.*/
    double rwSpeedOld[RW_EFF_CNT]; /*!< [r/s]  the RW spin rates from the prior control step */
    uint64_t priorTime;            /*!< [ns]   Last time the module control was called */
    int resetFlag;                 /*!< []     Flag indicating that a module reset occured */

    /* declare module IO interfaces */
    Message<RwMotorVoltageMsgPayload> voltageOutMsg;    /*!< voltage output message*/
    ReadFunctor<RwMotorTorqueMsgPayload> torqueInMsg;   /*!< Input torque message*/
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg; /*!< RW array input message*/
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;        /*!< [] The name for the reaction wheel speeds message. Must be
                                                           provided to enable speed tracking loop */
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg; /*!< [-] The name of the RWs availability message*/

    RWArrayConfigMsgPayload
        rwConfigParams; /*!< [-] struct to store message containing RW config parameters in body B frame */

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
