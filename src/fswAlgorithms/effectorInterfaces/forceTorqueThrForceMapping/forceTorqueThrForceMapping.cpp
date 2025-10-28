#include "forceTorqueThrForceMapping.h"
#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void ForceTorqueThrForceMapping::reset(uint64_t callTime) {
    if (!this->thrConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgPayload vehConfigInMsgBuffer = this->vehConfigInMsg();
    THRArrayConfigMsgPayload thrConfigInMsgBuffer = this->thrConfigInMsg();

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    this->numThrusters = thrConfigInMsgBuffer.numThrusters;
    this->CoM_B = cArrayAsEigenVector(vehConfigInMsgBuffer.CoM_B);
    if (this->numThrusters > MAX_EFF_CNT) {
        throw std::invalid_argument("forceTorqueThrForceMapping thruster configuration input message has a number of "
                                    "thrusters that is larger than MAX_EFF_CNT");
    }

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        this->rThruster_B.col(i) = cArrayAsEigenVector(thrConfigInMsgBuffer.thrusters[i].rThrust_B);
        this->gtThruster_B.col(i) = cArrayAsEigenVector(thrConfigInMsgBuffer.thrusters[i].tHatThrust_B);
        if (thrConfigInMsgBuffer.thrusters[i].maxThrust <= 0.0) {
            throw std::invalid_argument("forceTorqueThrForceMapping: A configured thruster has a non-sensible "
                                        "saturation limit of <= 0 N!");
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void ForceTorqueThrForceMapping::updateState(uint64_t callTime) {
    CmdTorqueBodyMsgPayload cmdTorqueInMsgBuffer{};
    CmdForceBodyMsgPayload cmdForceInMsgBuffer{};
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer{};

    /* Check if torque message is linked and read, zero out if not*/
    if (this->cmdTorqueInMsg.isLinked()) {
        cmdTorqueInMsgBuffer = this->cmdTorqueInMsg();
    }

    /* Check if force message is linked and read, zero out if not*/
    if (this->cmdForceInMsg.isLinked()) {
        cmdForceInMsgBuffer = this->cmdForceInMsg();
    }

    /* Create the torque and force vector */
    Eigen::Vector<double, 6> forceTorque_B{};
    forceTorque_B.head(3) = cArrayAsEigenVector(cmdTorqueInMsgBuffer.torqueRequestBody);
    forceTorque_B.tail(3) = cArrayAsEigenVector(cmdForceInMsgBuffer.forceRequestBody);

    /* - compute thruster locations relative to COM */
    Eigen::Matrix<double, 3, MAX_EFF_CNT> rThrusterRelCOM_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};
    rThrusterRelCOM_B.leftCols(this->numThrusters) = this->rThruster_B.leftCols(this->numThrusters).colwise() - this->CoM_B;

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
    uint32_t numRows = nonZeroRows;
    uint32_t numCols = this->numThrusters;

    Eigen::Vector<double, MAX_EFF_CNT> force_B{Eigen::Vector<double, MAX_EFF_CNT>::Zero()};
    force_B.topRows(numCols) = DG_nonzero.topLeftCorner(numRows, numCols).transpose() *
        (DG_nonzero.topLeftCorner(numRows, numCols) * DG_nonzero.topLeftCorner(numRows, numCols).transpose()).inverse() *
            forceTorque_B_nonzero.topRows(numRows);

    /* Find the minimum force */
    double minForce = force_B.topRows(this->numThrusters).minCoeff();

    /* Subtract the minimum force */
    Eigen::Vector<double, MAX_EFF_CNT> forceSubtracted_B{Eigen::Vector<double, MAX_EFF_CNT>::Zero()};
    forceSubtracted_B.topRows(this->numThrusters) = force_B.topRows(this->numThrusters).array() - minForce;

    /* Write to the output messages */
    eigenVectorToCArray(forceSubtracted_B, thrForceCmdOutMsgBuffer.thrForce);
    this->thrForceCmdOutMsg.write(&thrForceCmdOutMsgBuffer, this->moduleID, callTime);
}
