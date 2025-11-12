#ifndef _XMERA_MRP_PD_
#define _XMERA_MRP_PD_

#include <stdint.h>
#include <stdexcept>

#include <Eigen/Dense>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include "mrpPDAlgorithm.h"

/*! @brief MRP PD control class. */
class MrpPD : public SysModel {
   public:
    MrpPD() = default;
    ~MrpPD() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setDerivativeGainP(double P);
    double getDerivativeGainP() const;
    void setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B);
    const Eigen::Vector3d& getKnownTorquePntB_B() const;
    void setProportionalGainK(double K);
    double getProportionalGainK() const;

    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< Commanded torque output message

   private:
    MrpPDAlgorithm algorithm{};  //!< Algorithm for mrpPD control logic (BSK-agnostic)
};

#endif
