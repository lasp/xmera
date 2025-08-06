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

#ifndef _MRP_PD_ALGORITHM_H_
#define _MRP_PD_ALGORITHM_H_

#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/VehicleConfigMsgPayload.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief MRP PD control algorithm class. */
class MrpPDAlgorithm {
   public:
    MrpPDAlgorithm() = default;   //!< Constructor
    ~MrpPDAlgorithm() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos, VehicleConfigMsgPayload vehConfigInMsg);  //!< Reset member function
    CmdTorqueBodyMsgPayload update(uint64_t currentSimNanos,
                                   AttGuidMsgPayload guidInMsg);  //!< Update member function
    double getDerivativeGainP() const;                            //!< Getter method for derivative gain P
    const Eigen::Vector3d& getKnownTorquePntB_B() const;  //!< Getter method for the known external torque about point B
    double getProportionalGainK() const;                  //!< Getter method for proportional gain K
    void setDerivativeGainP(double P);                    //!< Setter method for derivative gain P
    void setKnownTorquePntB_B(
        Eigen::Vector3d& knownTorquePntB_B);  //!< Setter method for the known external torque about point B
    void setProportionalGainK(double K);      //!< Setter method for proportional gain K

   private:
    double K{};                           //!< [rad/s] Proportional gain applied to MRP errors
    double P{};                           //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3d knownTorquePntB_B{};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3d ISCPntB_B =
        Eigen::Matrix3d::Identity();  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
