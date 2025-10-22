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

#include "rateServoFullNonlinearAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <math.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param vehConfigMsg vehicle config message
 @param rwConfigMsg reaction wheel config message
 @param rwIsLinked boolean indicating whether reaction wheel config message is linked
 */
void RateServoFullNonlinearAlgorithm::reset(VehicleConfigMsgPayload vehConfigMsg,
                                            RWArrayConfigMsgPayload rwConfigMsg,
                                            bool rwIsLinked) {
    this->ISCPntB_B = cArrayAsEigenMatrix3(vehConfigMsg.ISCPntB_B);

    this->rwConfigParams.numRW = 0;
    if (rwIsLinked) {
        this->rwConfigParams = rwConfigMsg;
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
 @param callTime The clock time at which the function was called (nanoseconds)
 @param guidCmd Attitude tracking error message
 @param rateCmd Rate steering law message
 @param wheelSpeeds Reaction wheel speed message
 @param wheelsAvailability Reaction wheel availability message
 */
CmdTorqueBodyMsgPayload RateServoFullNonlinearAlgorithm::update(uint64_t callTime,
                                                                AttGuidMsgPayload guidCmd,
                                                                RateCmdMsgPayload rateCmd,
                                                                RWSpeedMsgPayload wheelSpeeds,
                                                                RWAvailabilityMsgPayload wheelsAvailability) {
    CmdTorqueBodyMsgPayload controlOut{}; /*!< commanded torque output message */

    /*! - compute control update time */
    double dt; /* [s] control update period */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    Eigen::Vector3d omega_BR_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_BR_B);
    Eigen::Vector3d omega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.omega_RN_B);
    Eigen::Vector3d domega_RN_B = Eigen::Map<const Eigen::Vector3d>(guidCmd.domega_RN_B);

    Eigen::Vector3d omega_BastR_B = Eigen::Map<const Eigen::Vector3d>(rateCmd.omega_BastR_B);
    Eigen::Vector3d omegap_BastR_B = Eigen::Map<const Eigen::Vector3d>(rateCmd.omegap_BastR_B);

    /*! - compute body rate */
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    /*! - compute the rate tracking error */
    Eigen::Vector3d omega_BastN_B = omega_BastR_B + omega_RN_B;
    Eigen::Vector3d omega_BBast_B = omega_BN_B - omega_BastN_B;

    /*! - integrate rate tracking error  */
    if (this->Ki > 0) { /* check if integral feedback is turned on  */
        this->z += omega_BBast_B * dt;
        for (uint32_t i = 0; i < 3; i++) {
            double intLimCheck = fabs(this->z[i]);
            if (intLimCheck > this->integralLimit) {
                this->z[i] *= this->integralLimit / intLimCheck;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        this->z = Eigen::Vector3d::Zero();
    }

    /*! - evaluate required attitude control torque Lr */
    Eigen::Vector3d Lr = this->P * omega_BBast_B + this->Ki * this->z;

    Eigen::Matrix<double, 3, RW_EFF_CNT> G_s_B =
        cArrayAsEigenMatrix<double, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);

    Eigen::Vector3d H_B = this->ISCPntB_B * omega_BN_B;
    for (uint32_t i = 0; i < this->rwConfigParams.numRW; i++) {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) { /* check if wheel is available */
            Eigen::Vector3d G_s_B_i = G_s_B.col(i);
            Eigen::Vector3d h_s_i =
                this->rwConfigParams.JsList[i] * (omega_BN_B.dot(G_s_B_i) + wheelSpeeds.wheelSpeeds[i]) * G_s_B_i;
            H_B += h_s_i;
        }
    }
    Lr -= omega_BastN_B.cross(H_B);

    Lr += -this->ISCPntB_B * (omegap_BastR_B + domega_RN_B - omega_BN_B.cross(omega_RN_B)) + this->knownTorquePntB_B;

    /* Change sign to compute the net positive control torque onto the spacecraft */
    Eigen::Vector3d u_s = -Lr;

    /*! - Set output message and pass it to the message bus */
    eigenVectorToCArray(u_s, controlOut.torqueRequestBody);

    return controlOut;
}

/*! Setter method for the gain P.
 @return void
 @param gain [N*m*s] Rate error feedback gain
*/
void RateServoFullNonlinearAlgorithm::setP(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("Feedback gain P must not be negative");
    }
    this->P = gain;
}

/*! Getter method for the gain P.
 @return const double
*/
double RateServoFullNonlinearAlgorithm::getP() const { return this->P; }

/*! Setter method for the gain Ki.
 @return void
 @param gain [N*m] Integral feedback gain
*/
void RateServoFullNonlinearAlgorithm::setKi(const double gain) { this->Ki = gain; }

/*! Getter method for the gain Ki.
 @return const double
*/
double RateServoFullNonlinearAlgorithm::getKi() const { return this->Ki; }

/*! Setter method for the integral limit.
 @return void
 @param limit [N*m*s] Integral limit
*/
void RateServoFullNonlinearAlgorithm::setIntegralLimit(const double limit) { this->integralLimit = limit; }

/*! Getter method for the integral limit.
 @return const double
*/
double RateServoFullNonlinearAlgorithm::getIntegralLimit() const { return this->integralLimit; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateServoFullNonlinearAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
Eigen::Vector3d RateServoFullNonlinearAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
