/*! @brief Top level structure for the thrust CM estimation kalman filter.
 Used to estimate the spacecraft's center of mass position with respect to the B frame.
 */

#ifndef THRUSTCMESTIMATION_H
#define THRUSTCMESTIMATION_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CMEstDataMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <math.h>
#include <string.h>
#include <array>

class ThrustCMEstimation : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    /*! declare these user-defined quantities */
    double attitudeTol;

    ReadFunctor<THRConfigMsgPayload> thrusterConfigBInMsg;        //!< thr config in msg in B-frame coordinates
    ReadFunctor<CmdTorqueBodyMsgPayload> intFeedbackTorqueInMsg;  //!< integral feedback torque input msg
    ReadFunctor<AttGuidMsgPayload> attGuidInMsg;                  //!< attitude guidance input msg
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;          //!< (optional) vehicle configuration input msg
    Message<CMEstDataMsgPayload> cmEstDataOutMsg;                 //!< estimated CM output msg
    Message<VehicleConfigMsgPayload> vehConfigOutMsg;             //!< output C++ vehicle configuration msg

    Eigen::Vector3d r_CB_B;  //!< initial CM estimate
    Eigen::Vector3d P0;      //!< initial CM state covariance
    Eigen::Vector3d R0;      //!< measurement noise covariance

   private:
    Eigen::Matrix3d I;         //!< identity matrix
    Eigen::Matrix3d P;         //!< state covariance
    Eigen::Matrix3d R;         //!< measurement noise covariance
    Eigen::Vector3d r_CB_est;  //!< CM location estimate

    bool cmKnowledge;  //!< boolean to assess if vehConfigInMsg is connected

    BSKLogger bskLogger{};  //!< -- BSK Logging
};

#endif
