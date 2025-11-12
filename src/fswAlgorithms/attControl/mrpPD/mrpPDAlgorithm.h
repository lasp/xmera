#ifndef _MRP_PD_ALGORITHM_H_
#define _MRP_PD_ALGORITHM_H_

#include <stdint.h>
#include <stdexcept>

#include <Eigen/Dense>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

/*! @brief MRP PD control algorithm class. */
class MrpPDAlgorithm {
   public:
    MrpPDAlgorithm() = default;
    ~MrpPDAlgorithm() = default;

    CmdTorqueBodyMsgPayload update(uint64_t currentSimNanos, AttGuidMsgPayload guidInMsg);
    void setSpacecraftInertia(VehicleConfigMsgPayload vehConfigInMsg);
    void setDerivativeGainP(double P);
    double getDerivativeGainP() const;
    void setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B);
    const Eigen::Vector3d& getKnownTorquePntB_B() const;
    void setProportionalGainK(double K);
    double getProportionalGainK() const;

   private:
    double K{};                           //!< [rad/s] Proportional gain applied to MRP errors
    double P{};                           //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3d knownTorquePntB_B{};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3d ISCPntB_B =
        Eigen::Matrix3d::Identity();  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
