/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _BASILISK_MRP_PD_
#define _BASILISK_MRP_PD_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/VehicleConfigMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/attControl/mrpPD/mrpPDAlgorithm.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief MRP PD control class. */
class MrpPD : public SysModel {
   public:
    MrpPD() = default;   //!< Constructor
    ~MrpPD() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    double getDerivativeGainP() const;                    //!< Getter method for derivative gain P
    const Eigen::Vector3d& getKnownTorquePntB_B() const;  //!< Getter method for the known external torque about point B
    double getProportionalGainK() const;                  //!< Getter method for proportional gain K
    void setDerivativeGainP(double P);                    //!< Setter method for derivative gain P
    void setKnownTorquePntB_B(
        Eigen::Vector3d& knownTorquePntB_B);  //!< Setter method for the known external torque about point B
    void setProportionalGainK(double K);      //!< Setter method for proportional gain K

    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< Commanded torque output message

    BSKLogger* bskLogger;  //!< BSK Logging

   private:
    MrpPDAlgorithm algorithm;  //!< Algorithm for mrpPD control logic (BSK-agnostic)
};

#endif
