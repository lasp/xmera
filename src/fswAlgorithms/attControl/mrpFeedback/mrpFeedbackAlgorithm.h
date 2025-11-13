#ifndef MRP_FEEDBACK_ALGORITHM_H
#define MRP_FEEDBACK_ALGORITHM_H

#include <stdint.h>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <Eigen/Core>

/*! structure containing the MRP feedback algorithm output */
typedef struct {
    CmdTorqueBodyMsgPayload controlOut;     /*!< control torque output */
    CmdTorqueBodyMsgPayload intFeedbackOut; /*!< integral feedback torque output */
} MrpFeedbackOutput;

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm {
   public:
    void reset(VehicleConfigMsgPayload vehConfigMsg, RWArrayConfigMsgPayload rwConfigMsg, bool rwIsLinked);
    MrpFeedbackOutput update(uint64_t callTime,
                             AttGuidMsgPayload guidCmd,
                             RWSpeedMsgPayload wheelSpeeds,
                             RWAvailabilityMsgPayload wheelsAvailability);

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

   private:
    double K{};              //!< [rad/sec] Proportional gain applied to MRP errors
    double P{};              //!< [N*m*s]   Rate error feedback gain applied
    double Ki{};             //!< [N*m]     Integration feedback error on rate error
    double integralLimit{};  //!< [N*m]     Integration limit to avoid wind-up issue
    int controlLawType{};    //!<           Flag to choose between the two control laws available
    Eigen::Vector3d knownTorquePntB_B{
        Eigen::Vector3d::Zero()};  //!< [N*m]     known external torque in body frame vector components
    uint64_t priorTime{};          //!< [ns]      Last time the attitude control is called
    Eigen::Vector3d int_sigma{};   //!< [s] integral of the MPR attitude error
    Eigen::Matrix3d ISCPntB_B{};   //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgPayload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
};

#endif
