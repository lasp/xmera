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

#include "fswAlgorithms/effectorInterfaces/thrForceMapping/thrForceMappingAlgorithm.h"

#include <math.h>

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"

int8_t asInt(ThrForceSign value) { return static_cast<int8_t>(value); }

void substractMin(Eigen::Vector<double, MAX_EFF_CNT>& F, uint32_t size);

double computeTorqueAngErr(Eigen::Matrix<double, 3, MAX_EFF_CNT> D,
                           const Eigen::Vector3d& BLr_B,
                           uint32_t numForces,
                           double epsilon,
                           Eigen::Vector<double, MAX_EFF_CNT> F,
                           Eigen::Vector<double, MAX_EFF_CNT> FMag);

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param thrConfigInMsg Thruster configuration message
 */
void ThrForceMappingAlgorithm::reset(uint64_t callTime, THRArrayConfigMsgPayload& thrConfigInMsg) {
    assert(this->numControlAxes > 0);

    THRArrayConfigMsgPayload localThrusterData = thrConfigInMsg;

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    this->numThrusters = localThrusterData.numThrusters;
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        this->rThruster_B.row(i) = Eigen::Map<Eigen::Vector3d>(localThrusterData.thrusters[i].rThrust_B);
        this->gtThruster_B.row(i) =
            Eigen::Map<Eigen::Vector3d>(localThrusterData.thrusters[i].tHatThrust_B).transpose();
        if (localThrusterData.thrusters[i].maxThrust <= 0.0) {
            assert("thruster has a non-sensible saturation limit of <= 0 N." && false);
        } else {
            this->thrForceMag(i) = localThrusterData.thrusters[i].maxThrust;
        }
    }
}

/*! The module takes a body frame torque vector and projects it onto available RCS or DV thrusters.
 @return THRArrayCmdForceMsgPayload
 @param callTime The clock time at which the function was called (nanoseconds)
 @param cmdTorqueInMsg Command torque input message
 @param vehConfigInMsg Vehicle configuration input message
 */
THRArrayCmdForceMsgPayload ThrForceMappingAlgorithm::update(uint64_t callTime,
                                                            CmdTorqueBodyMsgPayload& cmdTorqueInMsg,
                                                            VehicleConfigMsgPayload& vehConfigInMsg) {
    /*! - copy the request 3D attitude control torque vector */
    Eigen::Vector3d Lr_B =
        Eigen::Map<Eigen::Vector3d>(cmdTorqueInMsg.torqueRequestBody);  // [Nm] commanded control torque

    /*! - compute thruster locations relative to COM */
    /* Part 1 of Eq. 4 */
    Eigen::Matrix<double, MAX_EFF_CNT, 3> rThrusterRelCOM_B =
        this->rThruster_B.rowwise() - Eigen::Map<Eigen::Vector3d>(vehConfigInMsg.CoM_B).transpose();

    /*! - compute general thruster force mapping matrix */
    Eigen::Vector3d Lr_offset = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, MAX_EFF_CNT> D =
        Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero();  // [m] mapping matrix from thruster forces to body torque
    for (uint32_t i = 0; i < this->numThrusters; ++i) {
        Eigen::Vector3d rCrossGt = rThrusterRelCOM_B.row(i).cross(this->gtThruster_B.row(i)); /* Eq. 6 */
        D.col(i) = rCrossGt;
        /* Handles the case where there is translational motion imparted during off-pulsing*/
        if (this->thrForceSign == ThrForceSign::NEGATIVE) {
            /* Computing local torques from each thruster -- Individual terms in Eq. 7*/
            Eigen::Vector3d LrLocal = rCrossGt * this->thrForceMag(i); /* [Nm] Torque provided by individual thruster */
            Lr_offset = Lr_offset - LrLocal; /* Summing of individual torques -- Eq. 5 & Eq. 7 */
        }
    }

    Lr_B = Lr_B + Lr_offset;

    /*! Map the control torque onto the control axes
     * Note: Lr_B_Bar is projected only onto the available control axes.
     * i.e. if using DV thrusters with only 1 control axis,
     * Lr_B_Bar = [#, 0, 0]
     */
    Eigen::Vector3d Lr_B_Bar = this->controlAxes_B * Lr_B;  // [Nm] Control torque that we actually control

    // 1st iteration of finding a set of force vectors to implement the control torque
    Eigen::Vector<double, MAX_EFF_CNT> F =
        this->findMinimumNormForce(D, Lr_B_Bar, this->numThrusters); /* [N] vector of commanded thruster forces */

    /*! - Remove forces components that are contributing to the RCS Null space (this is due to the geometry of the
     * thrusters) */
    if (this->thrForceSign == ThrForceSign::POSITIVE) {
        substractMin(F, this->numThrusters);
    }

    if ((this->thrForceSign == ThrForceSign::NEGATIVE && this->numControlAxes < 3) || this->use2ndLoop) {
        // Array of flags indicating if this thruster is used for the Lr_j
        std::array<bool, MAX_EFF_CNT> thrusterUsed{};
        // Reduced mapping matrix
        Eigen::Matrix<double, 3, MAX_EFF_CNT> Dbar = Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero();  // [m]
        int counterPosForces = 0;  // counter for number of positive thruster forces
        for (uint32_t i = 0; i < this->numThrusters; ++i) {
            if (F(i) * asInt(this->thrForceSign) > 0) {
                thrusterUsed[i] = true; /* Eq. 11 */
                for (uint32_t j = 0; j < 3; ++j) {
                    Dbar(j, counterPosForces) = D(j, i); /* Eq. 12 */
                }
                counterPosForces += 1;
            }
        }

        // [N] vector of intermediate thruster forces
        Eigen::Vector<double, MAX_EFF_CNT> Fbar = this->findMinimumNormForce(Dbar, Lr_B_Bar, counterPosForces);
        if (this->thrForceSign == ThrForceSign::POSITIVE) {
            substractMin(Fbar, counterPosForces);
        }
        uint32_t c = 0;
        for (uint32_t i = 0; i < this->numThrusters; ++i) {
            if (thrusterUsed[i]) {
                F(i) = Fbar(c);
                c++;
            } else {
                F(i) = 0.0;
            }
        }
    }

    this->outTorqAngErr =
        computeTorqueAngErr(D, Lr_B_Bar, this->numThrusters, this->epsilon, F, this->thrForceMag); /* Eq. 16*/
    /*  check if the angle between the request and actual torque exceeds a limit.  If so, then uniformly scale
        all thruster forces values to not exceed saturation.
        If the angle threshold is negative, then this scaling is bypassed.*/
    if (this->outTorqAngErr > this->angErrThresh) {
        double maxFractUse = 0.0;  // ratio of maximum requested thruster force relative to maximum thruster limit
        for (uint32_t i = 0; i < this->numThrusters; ++i) {
            if (this->thrForceMag(i) > 0.0 &&
                std::fabs(F(i)) / this->thrForceMag(i) > maxFractUse) /* confirming that maxThrust > 0 */
            {
                maxFractUse = std::fabs(F(i)) / this->thrForceMag(i);
            }
        }
        /* only scale the requested thruster force if one or more thrusters are saturated */
        if (maxFractUse > 1.0) {
            F *= 1.0 / maxFractUse;
            this->outTorqAngErr =
                computeTorqueAngErr(D, Lr_B_Bar, this->numThrusters, this->epsilon, F, this->thrForceMag);
        }
    }

    /* store the output message */
    THRArrayCmdForceMsgPayload thrusterForceOut{};
    for (int i = 0; i < F.size(); ++i) {
        thrusterForceOut.thrForce[i] = F(i);
    }

    return thrusterForceOut;
}

/*!
 Take a stack of force values find the smallest value, and subtract if from all force values.  Here the smallest values
 will become zero, while other forces increase.  This assumes that the thrusters are aligned such that if all
 thrusters are firing, then no torque or force is applied.  This ensures only positive force values are computed.
 */
void substractMin(Eigen::Vector<double, MAX_EFF_CNT>& F, uint32_t size) {
    double minValue = 0.0;
    for (uint32_t i = 0; i < size; ++i) {
        if (F(i) < minValue) {
            minValue = F(i);
        }
    }
    for (uint32_t i = 0; i < size; ++i) {
        F(i) -= minValue;
    }
}

/*!
 * @brief Find the minimum norm solution to the least squares problem. Use a least square inverse to determine
 * the smallest set of thruster forces that yield the desired torque vector. Note that this routine does not
 * constrain yet the forces to be either positive or negative.
 * @param D 3x32 matrix that maps the thruster forces F[i] to the spacecraft torque.
 * @param Lr_B The vector representing the requested control torque.
 * @param numForces The number of thrusters to include in the minimum norm computation
 * @return A vector containing the thruster force solutions to the least squares problem.
 */
Eigen::Vector<double, MAX_EFF_CNT> ThrForceMappingAlgorithm::findMinimumNormForce(
    const Eigen::Matrix<double, 3, MAX_EFF_CNT>& D,
    const Eigen::Vector3d& Lr_B_Bar,
    uint32_t numForces) const {
    /* find [D].[D]^T */
    // [C].[D] matrix -- Thrusters in body frame mapped on control axes
    Eigen::Matrix<double, 3, MAX_EFF_CNT> CD = this->controlAxes_B * D;  // [m^2]
    Eigen::Matrix3d CDCDT = Eigen::Matrix3d::Identity();                 // [m^2]  [CD].[CD]^T matrix
    for (uint32_t i = 0; i < this->numControlAxes; ++i) {
        for (uint32_t j = 0; j < this->numControlAxes; ++j) {
            CDCDT(i, j) = 0.0;
            for (uint32_t k = 0; k < numForces; ++k) {
                CDCDT(i, j) += CD(i, k) * CD(j, k); /* Part of Eq. 9 */
            }
        }
    }

    Eigen::Matrix3d CDCDTInv = Eigen::Matrix3d::Zero(); /* [m^2]  ([CD].[CD]^T)^-1 matrix */
    if (CDCDT.determinant() > this->epsilon) {
        CDCDTInv = CDCDT.inverse();
    }  // else CDCDTInv is already initialized to zeros

    /* If fewer than 3 control axes, then the 1's along the diagonal of DDTInv will
     * not conflict with the mapping, as Lr_B_Bar contains the necessary zeros
     * to inhibit projection */
    Eigen::Vector3d CDCDTInvLr = CDCDTInv * Lr_B_Bar;
    Eigen::Vector<double, MAX_EFF_CNT> F = CD.transpose() * CDCDTInvLr; /* Eq. 15 */
    return F;
}

/*!
 * @brief  Determine the angle between the desired torque vector and the actual torque vector.
 * @param D 3x32 matrix that maps the thruster forces F[i] to the spacecraft torque.
 * @param BLr_B The vector representing the requested control torque.
 * @param numForces The number of thrusters to include in the computation
 * @param epsilon The threshold value for above which a vector norm is valid
 * @param F The force vectors which implement the control torque
 * @param FMag The force magnitudes of the thrusters
 * @return A vector containing the thruster force solutions to the least squares problem.
 */
double computeTorqueAngErr(Eigen::Matrix<double, 3, MAX_EFF_CNT> D,
                           const Eigen::Vector3d& BLr_B,
                           uint32_t numForces,
                           double epsilon,
                           Eigen::Vector<double, MAX_EFF_CNT> F,
                           Eigen::Vector<double, MAX_EFF_CNT> FMag) {
    double returnAngle = 0.0;  // [rad] angle between requested and actual torque vector
    /*! - make sure a control torque is requested, otherwise just return a zero angle error */
    if (BLr_B.norm() > epsilon) {
        Eigen::Matrix<double, MAX_EFF_CNT, 3> DT = D.transpose();
        Eigen::Vector3d BLr_hat_B = BLr_B.normalized();         // normalized BLr_B vector
        Eigen::Vector3d tauActual_B = Eigen::Vector3d::Zero();  // [Nm] control torque with current thruster solution

        /*! - loop over all thrusters and compute the actual torque to be applied */
        for (uint32_t i = 0; i < numForces; ++i) {
            /* This could produce inf's as F(i) approaches 0 if FMag(i) is 0, as such we
             * assert if FMag(i) is equal to zero in reset() */
            double thrusterForce = std::fabs(F(i)) < FMag(i) ? F(i)
                                                             : FMag(i) * std::fabs(F(i)) /
                                                                   F(i); /* [N] saturation constrained thruster force */
            Eigen::Vector3d LrEffector_B = DT.row(i) * thrusterForce;  // [Nm] torque of an individual thruster effector
            tauActual_B += LrEffector_B;
        }

        /*! - evaluate the angle between the requested and thruster implemented torque vector */
        tauActual_B.normalize();
        if (BLr_hat_B.dot(tauActual_B) < 1.0) {
            returnAngle = safeAcos(BLr_hat_B.dot(tauActual_B)); /* Eq 16 */
        }
    }
    return returnAngle;
}

/**
 * @brief Get the control axes in the body frame.
 * @return 3x3 matrix representing the control axes in the body frame.
 */
Eigen::Matrix3d ThrForceMappingAlgorithm::getControlAxesB() const { return this->controlAxes_B; }

/**
 * @brief Set the control axes in body frame.
 * @param axes A 3x3 matrix representing the control axes in body frame.
 */
void ThrForceMappingAlgorithm::setControlAxesB(const Eigen::Matrix3d& axes) {
    this->controlAxes_B = axes;
    this->numControlAxes = 0;
    for (uint32_t i = 0; i < 3; ++i) {
        if (this->controlAxes_B.col(i).norm() > this->epsilon) {
            this->controlAxes_B.col(i).normalize();
            this->numControlAxes++;
        } else {
            break;
        }
    }
}

/**
 * @brief Get the thruster force magnitudes.
 * @return A vector of thruster force magnitudes.
 */
Vector36d ThrForceMappingAlgorithm::getThrForceMag() const { return this->thrForceMag; }

/**
 * @brief Set the thruster force magnitudes.
 * @param forceMag A vector of thruster force magnitudes.
 */
void ThrForceMappingAlgorithm::setThrForceMag(const Vector36d& forceMag) { this->thrForceMag = forceMag; }

/**
 * @brief Get the sign of the thruster forces.
 * @return The sign of the thruster forces (POSITIVE or NEGATIVE).
 */
ThrForceSign ThrForceMappingAlgorithm::getThrForceSign() const { return this->thrForceSign; }

/**
 * @brief Set the sign of the thruster forces.
 * @param sign The sign of the thruster forces (POSITIVE or NEGATIVE).
 */
void ThrForceMappingAlgorithm::setThrForceSign(ThrForceSign sign) { this->thrForceSign = sign; }

/**
 * @brief Get the angular error threshold.
 * @return The angular error threshold.
 */
double ThrForceMappingAlgorithm::getAngErrThresh() const { return this->angErrThresh; }

/**
 * @brief Set the angular error threshold.
 * @param The new angular error threshold.
 */
void ThrForceMappingAlgorithm::setAngErrThresh(double thresh) { this->angErrThresh = thresh; }

/**
 * @brief Get the epsilon value.
 * @return The epsilon value.
 */
double ThrForceMappingAlgorithm::getEpsilon() const { return this->epsilon; }

/**
 * @brief Set the epsilon value.
 * @param The new epsilon value.
 */
void ThrForceMappingAlgorithm::setEpsilon(double eps) { this->epsilon = eps; }

/**
 * @brief Check if the second least squares fitting loop should be used.
 * @return True if the 2nd loop should be used, false otherwise.
 */
bool ThrForceMappingAlgorithm::getUse2ndLoop() const { return this->use2ndLoop; }

/**
 * @brief Set if the second least squares fitting loop should be used.
 * @return True if the 2nd loop should be used, false otherwise.
 */
void ThrForceMappingAlgorithm::setUse2ndLoop(bool loopFlag) { this->use2ndLoop = loopFlag; }
