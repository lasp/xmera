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
#include "simulation/sensors/starTracker/starTracker.h"

#include <iostream>

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/gauss_markov.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

StarTracker::StarTracker() {
    this->sensorTimeTag = 0;
    this->dcm_CB.setIdentity();
    this->errorModel = GaussMarkov(3, this->RNGSeed);
    this->PMatrix.fill(0.0);
    this->AMatrix.fill(0.0);
    this->walkBounds.fill(0.0);
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
    sigmaSensed = addMrp(cArray2EigenVector3d(this->scState.sigma_BN), this->mrpErrors);

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
    Eigen::Matrix3d dcm_CN; /* dcm, inertial to case frame */

    dcm_BN = mrpToDcm(*sigma);
    dcm_CN = this->dcm_CB * dcm_BN;

    Eigen::Vector4d beta_CN = dcmToEp(dcm_CN);
    eigenVector4d2CArray(beta_CN, sensorValues->qInrtl2Case);
}

/*!
    compute true output values
 */
void StarTracker::computeTrueOutput() {
    this->trueValues.timeTag = this->sensorTimeTag;
    Eigen::Vector3d sigma_BN = cArray2EigenVector3d(this->scState.sigma_BN);
    this->computeQuaternion(&sigma_BN, &this->trueValues);
}

/*!
    write output messages
 */
void StarTracker::writeOutputMessages(uint64_t currentSimNanos) {
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
    this->writeOutputMessages(currentSimNanos);
}
