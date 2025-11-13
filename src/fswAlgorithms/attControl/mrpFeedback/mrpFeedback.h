#ifndef MRP_FEEDBACK_H
#define MRP_FEEDBACK_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include "mrpFeedbackAlgorithm.h"

#include <Eigen/Core>

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedback : public SysModel {
   public:
    MrpFeedback() = default;
    ~MrpFeedback() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK(const double gain);
    double getK() const;
    void setP(const double gain);
    double getP() const;
    void setKi(const double gain);
    double getKi() const;
    void setIntegralLimit(const double limit);
    double getIntegralLimit() const;
    void setControlLawType(const int type);
    int getControlLawType() const;
    void setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B);
    Eigen::Vector3d getKnownTorquePntB_B() const;

    /* declare module IO interfaces */
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;        //!< RW speed input message (Optional)
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;  //!< RW availability input message (Optional)
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;  //!< RW parameter input message.  (Optional)
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;  //!< commanded spacecraft external control torque output message
    Message<CmdTorqueBodyMsgPayload>
        intFeedbackTorqueOutMsg;                          //!< commanded integral feedback control torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< vehicle configuration input message

   private:
    MrpFeedbackAlgorithm algorithm{};
    uint32_t numRW{};  //!< number of reaction wheels
};

#endif
