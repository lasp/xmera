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

#include <stdint.h>
#include <stdexcept>

#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/VehicleConfigMsgPayload.h"
#include "fswAlgorithms/attControl/mrpPD/mrpPDAlgorithm.h"

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
    MrpPDAlgorithm algorithm;  //!< Algorithm for mrpPD control logic (BSK-agnostic)
};

#endif
