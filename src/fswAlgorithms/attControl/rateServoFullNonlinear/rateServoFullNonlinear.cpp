/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
/*
    rateServoFullNonlinear Module

 */

#include "fswAlgorithms/attControl/rateServoFullNonlinear/rateServoFullNonlinear.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"

#include <string.h>
#include <math.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param this The configuration data associated with the servo rate control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
 */
void RateServoFullNonlinear::reset(uint64_t callTime)
{
    /*! - Read the input messages */
    VehicleConfigMsgPayload sc;

    /* make sure option msg connections are correctly done */
    if (this->rwParamsInMsg.isLinked()) {
        if (!this->rwSpeedsInMsg.isLinked()) {
            throw std::invalid_argument("rateServoFullNonlinear.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
        }
    }

    // check if essential messages are connected
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.guidInMsg wasn't connected.");
    }

    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.vehConfigInMsg wasn't connected.");
    }

    if (!this->rateSteeringInMsg.isLinked()) {
        throw std::invalid_argument("rateServoFullNonlinear.rateSteeringInMsg wasn't connected.");
    }


    sc = this->vehConfigInMsg();
    this->ISCPntB_B = cArrayAsEigenMatrix3(sc.ISCPntB_B);

    this->rwConfigParams.numRW = 0;
    if (this->rwParamsInMsg.isLinked()) {
        this->rwConfigParams = this->rwParamsInMsg();
    }

    /* Reset the integral measure of the rate tracking error */
    this->z = Eigen::Vector3d::Zero();

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;

}

/*! This method takes and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param this The configuration data associated with the servo rate control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
 */
void RateServoFullNonlinear::updateState(uint64_t callTime)
{
    AttGuidMsgPayload   guidCmd;                    /*!< Guidance input Message */
    RWSpeedMsgPayload   wheelSpeeds = {};           /*!< Reaction wheel speed estimates input message */
    RWAvailabilityMsgPayload wheelsAvailability = {};/*!< Reaction wheel availability input message */
    RateCmdMsgPayload   rateGuid;                   /*!< rate steering law message input message */
    CmdTorqueBodyMsgPayload controlOut = {};        /*!< commanded torque output message */

    double              dt;                 /* [s] control update period */

    /*! - compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    /*! - Zero and read the dynamic input messages */
    guidCmd = this->guidInMsg();
    rateGuid = this->rateSteeringInMsg();


    if(this->rwConfigParams.numRW > 0) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    Eigen::Vector3d omega_BR_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_BR_B);
    Eigen::Vector3d omega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_RN_B);
    Eigen::Vector3d domega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.domega_RN_B);

    Eigen::Vector3d omega_BastR_B = Eigen::Map<const Eigen::Vector3d>(rateGuid.omega_BastR_B);
    Eigen::Vector3d omegap_BastR_B = Eigen::Map<const Eigen::Vector3d>(rateGuid.omegap_BastR_B);

    /*! - compute body rate */
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    Eigen::Vector3d omega_BastN_B = omega_BastR_B + omega_RN_B;
    Eigen::Vector3d omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (this->Ki > 0) {   /* check if integral feedback is turned on  */
        this->z += omega_BBast_B * dt;
        for (uint32_t i=0; i<3; i++) {
            double intLimCheck = fabs(this->z[i]);
            if (intLimCheck > this->integralLimit) {
                this->z[i] *= this->integralLimit/intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        this->z = Eigen::Vector3d::Zero();
    }

    /*! - evaluate required attitude control torque Lr */
    Eigen::Vector3d Lr = this->P * omega_BBast_B + this->Ki * this->z;

    Eigen::Matrix<double, RW_EFF_CNT, 3> G_s_B{};
    G_s_B = (Eigen::Map<const Eigen::Matrix<double, 3, RW_EFF_CNT>>(this->rwConfigParams.GsMatrix_B, G_s_B.rows(), G_s_B.cols())).transpose();

    Eigen::Vector3d H_B = this->ISCPntB_B * omega_BN_B;
    for(uint32_t i = 0; i < this->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            Eigen::Vector3d G_s_B_i = G_s_B.row(i);
            Eigen::Vector3d h_s_i = this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;

        }
    }
    Lr -= omega_BastN_B.cross(H_B);

    Lr += - this->ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) + this->knownTorquePntB_B;

    /* Change sign to compute the net positive control torque onto the spacecraft */
    Eigen::Vector3d u_s = -Lr;

    /*! - Set output message and pass it to the message bus */
    eigenVectorToCArray(u_s, controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);

    return;
}
