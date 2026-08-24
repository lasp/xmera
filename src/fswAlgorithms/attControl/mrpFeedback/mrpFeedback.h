// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MRP_FEEDBACK_H
#define MRP_FEEDBACK_H

#include "mrpFeedbackAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <stdint.h>

#include <Eigen/Core>

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedback final : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK(double const gain);
    double getK() const;
    void setP(double const gain);
    double getP() const;
    void setKi(double const gain);
    double getKi() const;
    void setIntegralLimit(double const limit);
    double getIntegralLimit() const;
    void setControlLawType(int const type);
    int getControlLawType() const;
    void setKnownTorquePntB_B(Eigen::Vector3d const &knownTorquePntB_B);
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
