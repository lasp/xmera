#include "thrMomentumManagementCpp.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>

void ThrMomentumManagementCpp::reset(uint64_t currentSimNanos) {
    // Check if the required input messages are included
    if (!this->rwConfigDataInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "thrMomentumManagementCpp.rwConfigDataInMsg wasn't connected.");
    }
    if (!this->rwSpeedsInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "thrMomentumManagementCpp.rwSpeedsInMsg wasn't connected.");
    }

    // Read in the RW configuration message
    this->rwConfigParams = this->rwConfigDataInMsg();

    // Reset the momentum dumping request flag
    this->initRequest = 1;
}

void ThrMomentumManagementCpp::updateState(uint64_t currentSimNanos) {
    Eigen::Vector3d Delta_H_B = Eigen::Vector3d::Zero();

    if (this->initRequest == 1) {
        RWSpeedMsgPayload rwSpeedMsg = this->rwSpeedsInMsg();
        Eigen::Vector3d hs_B = Eigen::Vector3d::Zero();
        for (int i = 0; i < this->rwConfigParams.numRW; i++) {
            hs_B += this->rwConfigParams.JsList[i] * rwSpeedMsg.wheelSpeeds[i] *
                    cArrayToEigenVector3(&this->rwConfigParams.GsMatrix_B[i * 3]);
        }

        if (this->hd_B.norm() > 0) {
            Delta_H_B = this->hd_B - hs_B;
        } else {
            if (double hs = hs_B.norm(); hs >= this->hs_min) {
                Delta_H_B = -hs_B * (hs - this->hs_min) / hs;
            }
        }

        this->initRequest = 0;
    }

    CmdTorqueBodyMsgPayload controlOutMsg{};
    eigenVectorToCArray(Delta_H_B, controlOutMsg.torqueRequestBody);
    this->deltaHOutMsg.write(&controlOutMsg, this->moduleID, currentSimNanos);
}
