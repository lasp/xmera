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

#include "forceTorqueThrForceMappingAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>
#include <Eigen/QR>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param vehConfigMsg vehicle configuration message
 @param thrConfigMsg thruster configuration message
*/
void ForceTorqueThrForceMappingAlgorithm::reset(VehicleConfigMsgPayload& vehConfigMsg,
                                                THRArrayConfigMsgPayload& thrConfigMsg) {
    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    this->numThrusters = thrConfigMsg.numThrusters;
    this->CoM_B = cArrayToEigenVector(vehConfigMsg.CoM_B);
    if (this->numThrusters > MAX_EFF_CNT) {
        throw std::invalid_argument(
            "forceTorqueThrForceMapping thruster configuration input message has a number of "
            "thrusters that is larger than MAX_EFF_CNT");
    }

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        this->rThruster_B.col(i) = cArrayToEigenVector(thrConfigMsg.thrusters[i].rThrust_B);
        this->gtThruster_B.col(i) = cArrayToEigenVector(thrConfigMsg.thrusters[i].tHatThrust_B);
        if (thrConfigMsg.thrusters[i].maxThrust <= 0.0) {
            throw std::invalid_argument(
                "forceTorqueThrForceMapping: A configured thruster has a non-sensible "
                "saturation limit of <= 0 N!");
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param cmdTorqueMsg commanded torque message
 @param cmdForceMsg commanded force message
*/
THRArrayCmdForceMsgPayload ForceTorqueThrForceMappingAlgorithm::update(CmdTorqueBodyMsgPayload& cmdTorqueMsg,
                                                                       CmdForceBodyMsgPayload& cmdForceMsg) const {
    THRArrayCmdForceMsgPayload thrForceCmdOutMsg{};

    /* Create the torque and force vector */
    Eigen::Vector<double, 6> forceTorque_B{};
    forceTorque_B.head(3) = cArrayToEigenVector(cmdTorqueMsg.torqueRequestBody);
    forceTorque_B.tail(3) = cArrayToEigenVector(cmdForceMsg.forceRequestBody);

    /* - compute thruster locations relative to COM */
    Eigen::Matrix<double, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(this->numThrusters) =
        this->rThruster_B.leftCols(this->numThrusters).colwise() - this->CoM_B;

    /* Fill DG with thruster directions and moment arms */
    Eigen::Matrix<double, 3, MAX_EFF_CNT> rCrossGt{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        rCrossGt.col(i) = rThrusterRelCOM_B.col(i).cross(this->gtThruster_B.col(i));
    }
    Eigen::Matrix<double, 6, MAX_EFF_CNT> DG{};
    DG << rCrossGt, this->gtThruster_B;

    /* Create the DG w/ zero rows removed */
    uint32_t nonZeroRows = 0;
    Eigen::Matrix<double, 6, MAX_EFF_CNT> DG_nonzero{Eigen::Matrix<double, 6, MAX_EFF_CNT>::Zero()};
    Eigen::Vector<double, 6> forceTorque_B_nonzero{Eigen::Vector<double, 6>::Zero()};
    for (uint32_t i = 0; i < 6; ++i) {
        if ((DG.row(i).array().abs() > 1e-7).any()) {
            DG_nonzero.row(nonZeroRows) = DG.row(i);
            forceTorque_B_nonzero.row(nonZeroRows) = forceTorque_B.row(i);
            nonZeroRows += 1;
        }
    }

    /* Compute the force for each thruster */
    const uint32_t numRows = nonZeroRows;
    const uint32_t numCols = this->numThrusters;

    Eigen::Vector<double, MAX_EFF_CNT> force_B{Eigen::Vector<double, MAX_EFF_CNT>::Zero()};
    force_B.topRows(numCols) =
        DG_nonzero.topLeftCorner(numRows, numCols).completeOrthogonalDecomposition().pseudoInverse() *
        forceTorque_B_nonzero.topRows(numRows);

    /* Find the minimum force */
    const double minForce = force_B.topRows(this->numThrusters).minCoeff();

    /* Subtract the minimum force */
    Eigen::Vector<double, MAX_EFF_CNT> forceSubtracted_B{Eigen::Vector<double, MAX_EFF_CNT>::Zero()};
    forceSubtracted_B.topRows(this->numThrusters) = force_B.topRows(this->numThrusters).array() - minForce;

    eigenVectorToCArray(forceSubtracted_B, thrForceCmdOutMsg.thrForce);

    return thrForceCmdOutMsg;
}
