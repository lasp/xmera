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
    mrpFeedback Module

 */

#include "fswAlgorithms/attControl/mrpFeedback/mrpFeedback.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

#include <string.h>
#include <math.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::reset(uint64_t callTime)
{
    /* check that optional messages are correct connected */
    if(this->rwParamsInMsg.isLinked()) {
        if (!this->rwSpeedsInMsg.isLinked()) {
            throw std::invalid_argument("MrpFeedback.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
        }

    }

    // check if the required message has not been connected
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.vehConfigInMsg wasn't connected.");
    }

    /*! - zero and read in vehicle configuration message */
    VehicleConfigMsgPayload sc = this->vehConfigInMsg();
    /*! - copy over spacecraft inertia tensor */
    this->ISCPntB_B = cArrayAsEigenMatrix3(sc.ISCPntB_B);

    /*! - zero the number of RW by default */
    this->rwConfigParams.numRW = 0;

    /*! - check if RW configuration message exists */
    if (this->rwParamsInMsg.isLinked()) {
        /*! - Read static RW config data message and store it in module variables*/
        this->rwConfigParams = this->rwParamsInMsg();
    }

    /*! - Reset the integral measure of the rate tracking error */
    this->int_sigma = Eigen::Vector3d::Zero();

    /*! - Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpFeedback::updateState(uint64_t callTime)
{
    AttGuidMsgPayload      guidCmd;            /* attitude tracking error message */
    RWSpeedMsgPayload      wheelSpeeds;        /* Reaction wheel speed message */
    RWAvailabilityMsgPayload wheelsAvailability = {}; /* Reaction wheel availability message */
    CmdTorqueBodyMsgPayload controlOut = {};        /* output message */
    CmdTorqueBodyMsgPayload intFeedbackOut;    /* output int feedback msg */

    double              dt;                 /* [s] control update period */

    /*! - Read the attitude tracking error message */
    guidCmd = this->guidInMsg();

    /*! - read in optional RW speed and availability message */
    if(this->rwConfigParams.numRW > 0) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    /*! - compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    Eigen::Vector3d sigma_BR = Eigen::Map<const Eigen::Vector3d>(guidCmd.sigma_BR);
    Eigen::Vector3d omega_BR_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_BR_B);
    Eigen::Vector3d omega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_RN_B);
    Eigen::Vector3d domega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.domega_RN_B);

    /*! - compute body rate */
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - evaluate integral term */
    Eigen::Vector3d z{Eigen::Vector3d::Zero()};
    if (this->Ki > 0) {   /* check if integral feedback is turned on  */
        this->int_sigma += this->K * dt * sigma_BR;

        /* keep int_sigma less than integralLimit */
        for (uint32_t i=0; i<3; i++) {
            double intCheck = fabs(this->int_sigma[i]);
            if (intCheck > this->integralLimit) {
                this->int_sigma[i] *= this->integralLimit/intCheck;
            }
        }
        z = this->int_sigma + this->ISCPntB_B * omega_BR_B;
    }

    /*! - evaluate required attitude control torque Lr */
    Eigen::Vector3d Lr = this->K * sigma_BR + this->P * omega_BR_B + this->P * this->Ki * z;

    Eigen::Matrix<double, RW_EFF_CNT, 3> G_s_B{};
    G_s_B = (Eigen::Map<const Eigen::Matrix<double, 3, RW_EFF_CNT>>(this->rwConfigParams.GsMatrix_B, G_s_B.rows(), G_s_B.cols())).transpose();

    Eigen::Vector3d H_B = this->ISCPntB_B * omega_BN_B;
    for(uint32_t i=0; i<this->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            Eigen::Vector3d G_s_B_i = G_s_B.row(i);
            Eigen::Vector3d h_s_i = this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }

    if (this->controlLawType == 0) {
        Lr -= (omega_RN_B + this->Ki * z).cross(H_B);
    }
    else {
        Lr -= omega_BN_B.cross(H_B);
    }

    Lr += this->ISCPntB_B * (omega_BN_B.cross(omega_RN_B) - domega_RN_B) + this->knownTorquePntB_B;

    Eigen::Vector3d u_s = -Lr;
    Eigen::Vector3d u_integral = -(this->P * this->Ki * z);

    /*! - set the output message and write it out */
    eigenVectorToCArray(u_s, controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);

    /*! - write the output integral feedback torque */
    eigenVectorToCArray(u_integral, intFeedbackOut.torqueRequestBody);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);

    return;
}

/*! Setter method for the gain K.
 @return void
 @param gain [N*m] Attitude error feedback gain
*/
void MrpFeedback::setK(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("Feedback gain K must not be negative");
    }
    this->K = gain;
}

/*! Getter method for the gain K.
 @return const double
*/
double MrpFeedback::getK() const { return this->K; }

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void MrpFeedback::setP(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("Feedback gain P must not be negative");
    }
    this->P = gain;
}

/*! Getter method for the gain P.
 @return const double
*/
double MrpFeedback::getP() const { return this->P; }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void MrpFeedback::setKi(const double gain) { this->Ki = gain; }

/*! Getter method for the gain Ki.
 @return const double
*/
double MrpFeedback::getKi() const { return this->Ki; }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void MrpFeedback::setIntegralLimit(const double limit) { this->integralLimit = limit; }

/*! Getter method for the integral limit.
 @return const double
*/
double MrpFeedback::getIntegralLimit() const { return this->integralLimit; }

/*! Setter method for the control law type.
 @return void
 @param type control law type
*/
void MrpFeedback::setControlLawType(const int type) { this->controlLawType = type; }

/*! Getter method for the control law type.
 @return const int
*/
int MrpFeedback::getControlLawType() const { return this->controlLawType; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpFeedback::setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
Eigen::Vector3d MrpFeedback::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
