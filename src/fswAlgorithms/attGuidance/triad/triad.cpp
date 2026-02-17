// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "triad.h"

#include <math.h>

#include <Eigen/Core>

#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

const double epsilon = 1e-12;

double SPE_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double dot = v1.dot(v2);

    double cross = v1.x() * v2.y() - v1.y() * v2.x();

    // Compute angle in radians and convert to degrees
    double angle = std::acos(std::clamp(dot / (v1.norm() * v2.norm()), -1.0, 1.0));
    angle = angle * 180.0 / M_PI;

    if (cross < 0) {
        angle = -angle;
    }

    return angle;
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] time the method is called
*/
void Triad::reset(uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, " triad.attNavInMsg wasn't connected.");
    }

    // check how the input body heading is provided
    if (this->bodyHeadingInMsg.isLinked()) {
        this->bodyAxisInput = BodyAxisInput::inputBodyHeadingMsg;
    } else if (this->h1Hat_B.norm() > epsilon) {
        this->bodyAxisInput = BodyAxisInput::inputBodyHeadingParameter;
    } else {
        this->bskLogger.bskLog(BSK_ERROR,
                               " triad.bodyHeadingInMsg wasn't connected and no body heading h1Hat_B was specified.");
    }

    // check how the input inertial heading is provided
    if (this->inertialHeadingInMsg.isLinked()) {
        this->inertialAxisInput = InertialAxisInput::inputInertialHeadingMsg;
        if (this->ephemerisInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_WARNING,
                                   " both triad.inertialHeadingInMsg and triad.ephemerisInMsg were linked. Inertial "
                                   "heading is computed from triad.inertialHeadingInMsg");
        }
    } else if (this->ephemerisInMsg.isLinked()) {
        if (!this->transNavInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, " triad.ephemerisInMsg was specified but triad.transNavInMsg was not.");
        } else {
            this->inertialAxisInput = InertialAxisInput::inputEphemerisMsg;
        }
    } else {
        if (this->hHat_N.norm() > epsilon) {
            this->inertialAxisInput = InertialAxisInput::inputInertialHeadingParameter;
        } else {
            this->bskLogger.bskLog(BSK_ERROR,
                                   " neither triad.inertialHeadingInMsg nor triad.ephemerisInMsg were "
                                   "connected and no inertial heading h_N was specified.");
        }
    }
}

/*! The Update() function computes the reference MRP attitude, reference angular rate and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void Triad::updateState(uint64_t callTime) {
    /*! create and zero the output message */
    AttRefMsgPayload attRefOut = {};

    /*! read and allocate the attitude navigation message */
    NavAttMsgPayload attNavIn = this->attNavInMsg();

    /*! get requested heading in inertial frame */
    Eigen::Vector3d hReqHat_N;
    if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingParameter) {
        hReqHat_N = this->hHat_N.normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputInertialHeadingMsg) {
        InertialHeadingMsgPayload inertialHeadingIn = this->inertialHeadingInMsg();
        hReqHat_N = cArrayToEigenVector(inertialHeadingIn.rHat_XN_N).normalized();
    } else if (this->inertialAxisInput == InertialAxisInput::inputEphemerisMsg) {
        EphemerisMsgPayload ephemerisIn = this->ephemerisInMsg();
        NavTransMsgPayload transNavIn = this->transNavInMsg();
        hReqHat_N =
            (cArrayToEigenVector(ephemerisIn.r_BdyZero_N) - cArrayToEigenVector(transNavIn.r_BN_N)).normalized();
    }

    /*! get body frame heading */
    Eigen::Vector3d hRefHat_B;
    if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingParameter) {
        hRefHat_B = this->h1Hat_B.normalized();
    } else if (this->bodyAxisInput == BodyAxisInput::inputBodyHeadingMsg) {
        BodyHeadingMsgPayload bodyHeadingIn = this->bodyHeadingInMsg();
        hRefHat_B = cArrayToEigenVector(bodyHeadingIn.rHat_XB_B).normalized();
    }

    Eigen::MRPd sigma_BN(cArrayToEigenVector(attNavIn.sigma_BN));
    /*! define the body frame orientation DCM BN */
    Eigen::Matrix3d BN = sigma_BN.toRotationMatrix().transpose();

    /*! get the solar array drive direction in body frame coordinates */
    Eigen::Vector3d a1Hat_B = this->a1Hat_B.normalized();

    /*! read Sun direction in B frame from the attNav message */
    Eigen::Vector3d rHat_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy).normalized();

    Eigen::Vector3d rHat_SB_N;
    rHat_SB_N = BN.transpose() * rHat_SB_B;

    if (double SPE = SPE_angle(rHat_SB_N, hReqHat_N); std::abs(SPE) < 0.5) {
        this->bskLogger.bskLog(BSK_ERROR, " sun and earth reference vectors are parallel, Triad can not be used");
    }

    Eigen::Matrix3d BD;
    Eigen::Matrix3d RD;

    Eigen::Vector3d r2 = hRefHat_B;
    Eigen::Vector3d r3 = a1Hat_B.cross(hRefHat_B).normalized();

    Eigen::Vector3d r1 = r2.cross(r3);
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    Eigen::Vector3d n2 = hReqHat_N;
    Eigen::Vector3d n1 = rHat_SB_N.cross(hReqHat_N).normalized();
    Eigen::Vector3d n3 = n1.cross(n2);
    BD.col(0) = n1;
    BD.col(1) = n2;
    BD.col(2) = n3;

    Eigen::Matrix3d RN = RD * BD.transpose();

    /*! compute reference MRP */
    Eigen::Vector3d sigma_RN;
    sigma_RN = dcmToMrp(RN);

    double Sigma_RN[3];
    eigenVectorToCArray(sigma_RN, Sigma_RN);
    v3Copy(Sigma_RN, attRefOut.sigma_RN);

    /*! write output message */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/*! Setter method for sigma_R0R.
 @return void
 @param a1Hat_B
*/
void Triad::setA1Hat_B(const Eigen::Vector3d& a1Hat_B) { this->a1Hat_B = a1Hat_B; }
/*! Setter method for sigma_R0R.
 @return void
 @param h1Hat_B
*/
void Triad::setH1Hat_B(const Eigen::Vector3d& h1Hat_B) { this->h1Hat_B = h1Hat_B; }
/*! Setter method for sigma_R0R.
 @return void
 @param hHat_N
*/
void Triad::setHHat_N(const Eigen::Vector3d& hHat_N) { this->hHat_N = hHat_N; }
/*! Setter method for sigma_R0R.
 @return void
 @param celestialBodyInput
*/
void Triad::setCelestialBodyInput(const CelestialBody& celestialBodyInput) {
    this->celestialBodyInput = celestialBodyInput;
}
/*! Setter method for sigma_R0R.
 @return void
 @param bodyAxisInput
*/
void Triad::setBodyAxisInput(const BodyAxisInput& bodyAxisInput) { this->bodyAxisInput = bodyAxisInput; }
/*! Setter method for sigma_R0R.
 @return void
 @param inertialAxisInput
*/
void Triad::setInertialAxisInput(const InertialAxisInput& inertialAxisInput) {
    this->inertialAxisInput = inertialAxisInput;
}

/*! Getter method for a1Hat_B.
    @return const Eigen::Vector3d
*/
const Eigen::Vector3d Triad::getA1Hat_B() const { return this->a1Hat_B; }
/*! Getter method for h1Hat_B.
    @return const Eigen::Vector3d
*/
const Eigen::Vector3d Triad::getH1Hat_B() const { return this->h1Hat_B; }
/*! Getter method for hHat_N.
    @return const Eigen::Vector3d
*/
const Eigen::Vector3d Triad::getHHat_N() const { return this->hHat_N; }
/*! Getter method for celestialBodyInput.
    @return const CelestialBody
*/
const CelestialBody Triad::getCelestialBodyInput() const { return this->celestialBodyInput; }
/*! Getter method for bodyAxisInput.
    @return const BodyAxisInput
*/
const BodyAxisInput Triad::getBodyAxisInput() const { return this->bodyAxisInput; }
/*! Getter method for inertialAxisInput.
    @return const InertialAxisInput
*/
const InertialAxisInput Triad::getInertialAxisInput() const { return this->inertialAxisInput; }
