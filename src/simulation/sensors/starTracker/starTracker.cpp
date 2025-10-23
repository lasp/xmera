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
#include "starTracker.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/gauss_markov.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

StarTracker::StarTracker() {
    this->dcm_CB.setIdentity();
    this->errorModel = GaussMarkov(3, this->RNGSeed);
    this->PMatrix.fill(0.0);
    this->AMatrix.fill(0.0);
    return;
}

StarTracker::~StarTracker() { return; }

/*! This method is used to reset the module.
 @param currentSimNanos The current simulation time from the architecture
 @return void */
void StarTracker::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "starTracker.scStateInMsg was not linked.");
    }

    int numStates = 3;

    this->AMatrix.setIdentity(numStates, numStates);

    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if (this->PMatrix.size() != numStates * numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 3*3. Quitting.");
        return;
    }
    if (this->walkBounds.size() != numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds is not size 3. Quitting");
        return;
    }
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
}

/*!
    read input messages
 */
void StarTracker::readInputMessages() {
    this->scState = this->scStateInMsg();
    this->sensorTimeTag = this->scStateInMsg.timeWritten();
}

/*!
   compute sensor errors
 */
void StarTracker::computeSensorErrors() {
    this->errorModel.setPropMatrix(this->AMatrix);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

/*!
   apply sensor errors
 */
void StarTracker::applySensorErrors() {
    this->mrpErrors = prvToMrp(this->navErrors);

    Eigen::Vector3d sigmaSensed;
    sigmaSensed = addMrp(cArrayAsEigenVector(this->scState.sigma_BN), this->mrpErrors);

    // Save the previous sensed quaternion before computing the current sensed quaternion
    this->betaPrevious_CN = cArrayAsEigenVector(this->sensedValues.qInrtl2Case);

    this->computeQuaternion(&sigmaSensed, &this->sensedValues);
    this->sensedValues.timeTag = this->sensorTimeTag;
}

/*!
    compute quaternion from MRPs
    @param sigma
    @param sensorValues
 */
void StarTracker::computeQuaternion(Eigen::Vector3d* sigma, STSensorMsgPayload* sensorValues) {
    Eigen::Matrix3d dcm_BN; /* dcm, inertial to body frame */
    dcm_BN = mrpToDcm(*sigma);

    Eigen::Matrix3d dcm_CN; /* dcm, inertial to case frame */
    dcm_CN = this->dcm_CB * dcm_BN;

    Eigen::Vector4d beta_CN = dcmToEp(dcm_CN);
    eigenVectorToCArray(beta_CN, sensorValues->qInrtl2Case);
}

/*!
    compute platform angular velocity from sensed quaternions
 */
void StarTracker::computeAngularVelocity(uint64_t currentSimNanos) {
    Eigen::Vector4d beta_CN = cArrayAsEigenVector(this->sensedValues.qInrtl2Case);

    // Determine betaDot_CN
    Eigen::Vector4d betaDot_CN = Eigen::Vector4d::Zero();
    if (currentSimNanos != this->previousSimTime) {
        double dt = (currentSimNanos - this->previousSimTime) * NANO2SEC;
        betaDot_CN = (beta_CN - this->betaPrevious_CN) / dt;
    }

    Eigen::Matrix<double, 3, 4> bInv = binvEp(beta_CN);

    Eigen::Vector3d omega_CN_C = 2 * bInv * betaDot_CN;
    eigenVectorToCArray(omega_CN_C, this->sensedValues.omega_CN_C);
}

/*!
    compute true output values
 */
void StarTracker::computeTrueOutput() {
    this->trueValues.timeTag = this->sensorTimeTag;
    Eigen::Vector3d sigma_BN = cArrayAsEigenVector(this->scState.sigma_BN);
    this->computeQuaternion(&sigma_BN, &this->trueValues);
}

/*!
    write output messages
 */
void StarTracker::writeOutputMessages(uint64_t currentSimNanos) {
    this->sensedValues.timeTag = currentSimNanos * NANO2SEC;
    this->sensorOutMsg.write(&this->sensedValues, this->moduleID, currentSimNanos);
}

/*!
    update module states
 */
void StarTracker::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
    this->computeSensorErrors();
    this->computeTrueOutput();
    this->applySensorErrors();
    this->computeAngularVelocity(currentSimNanos);
    this->writeOutputMessages(currentSimNanos);
    this->previousSimTime = currentSimNanos;
}

/*! Setter method for dcm_CB.
 @return void
 @param dcm_CB
*/
void StarTracker::setDcmCB(const Eigen::Matrix3d& dcm_CB) { this->dcm_CB = dcm_CB; }

/*! Setter method for PMatrix.
 @return void
 @param PMatrix
*/
void StarTracker::setPMatrix(const Eigen::Matrix3d& PMatrix) { this->PMatrix = PMatrix; }

/*! Setter method for walkBounds.
 @return void
 @param walkBounds
*/
void StarTracker::setWalkBounds(const Eigen::Vector3d& walkBounds) { this->walkBounds = walkBounds; }

/*! Getter method for dcm_CB.
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d& StarTracker::getDcmCB() const { return this->dcm_CB; }

/*! Getter method for PMatrix.
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d& StarTracker::getPMatrix() const { return this->PMatrix; }

/*! Getter method for walkBounds.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& StarTracker::getWalkBounds() const { return this->walkBounds; }
