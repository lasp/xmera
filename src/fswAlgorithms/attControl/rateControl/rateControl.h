/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _RATE_CONTROL_
#define _RATE_CONTROL_

#include <stdint.h>

#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Rate control class. */
class RateControl : public SysModel {
   public:
    RateControl() = default;   //!< Constructor
    ~RateControl() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    const double getDerivativeGainP() const;              //!< Getter method for derivative gain P
    const Eigen::Vector3d &getKnownTorquePntB_B() const;  //!< Getter method for the known external torque about point B
    void setDerivativeGainP(const double P);              //!< Setter method for derivative gain P
    void setKnownTorquePntB_B(
        const Eigen::Vector3d &knownTorquePntB_B);  //!< Getter method for the known external torque about point B

    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< Commanded torque output message

    BSKLogger *bskLogger;  //!< BSK Logging

   private:
    double P{};                           //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3d knownTorquePntB_B{};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3d ISCPntB_B{
        Eigen::Matrix3d::Identity()};  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
