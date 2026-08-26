// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "magnetometer.h"

#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.h>

#include <math.h>

/*! This is the constructor, setting variables to default values. */
Magnetometer::Magnetometer() {
    this->numStates = 3;
    this->senBias.fill(0.0);       // Tesla
    this->senNoiseStd.fill(-1.0);  // Tesla
    this->walkBounds.fill(0.0);
    this->noiseModel = GaussMarkov<3>();
    this->scaleFactor = 1.0;
    this->maxOutput = 1e200;   // Tesla
    this->minOutput = -1e200;  // Tesla
    this->dcm_SB.setIdentity(3, 3);
    return;
}

/*! This is the destructor, nothing to report here. */
Magnetometer::~Magnetometer() {
    return;
}

//! - This method composes the transformation matrix from Body to Sensor frame.
Eigen::Matrix3d Magnetometer::setBodyToSensorDCM(double yaw, double pitch, double roll) {
    this->dcm_SB = eigenM1(roll) * eigenM2(pitch) * eigenM3(yaw);

    return this->dcm_SB;
}

/*! This method is used to reset the module.
 @param currentSimNanos The current simulation time from the architecture
 @return void */
void Magnetometer::reset(uint64_t currentSimNanos) {
    if (!this->magInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Magnetic field interface message name (magInMsg) is empty.");
    }

    if (!this->stateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Spacecraft state message name (stateInMsg) is empty.");
    }

    this->noiseModel.setUpperBounds(this->walkBounds);
    Eigen::Matrix3d nMatrix = (this->senNoiseStd * 1.5).asDiagonal();
    this->noiseModel.setNoiseMatrix(nMatrix);
    this->noiseModel.setRNGSeed(this->RNGSeed);
    return;
}

/*! This method reads necessary input messages. */
void Magnetometer::readInputMessages() {
    //! - Read magnetic field model ephemeris message
    this->magData = this->magInMsg();

    //! - Read vehicle state ephemeris message
    this->stateCurrent = this->stateInMsg();
}

/*! This method computes the magnetic field vector information in the sensor frame.*/
void Magnetometer::computeMagData() {
    Eigen::Vector3d tam_N;
    Eigen::Matrix3d dcm_BN;
    Eigen::MRPd sigma_BN;
    //! - Magnetic field vector in inertial frame using a magnetic field model (WMM, Dipole, etc.)
    tam_N = cArrayToEigenVector(this->magData.magField_N);
    sigma_BN = cArrayToEigenMrp(this->stateCurrent.sigma_BN);
    //! - Get the inertial to sensor frame transformation information and convert tam_N to tam_S
    dcm_BN = sigma_BN.toRotationMatrix().transpose();
    this->tam_S = this->dcm_SB * dcm_BN * tam_N;
}

/*! This method computes the true sensed values for the sensor. */
void Magnetometer::computeTrueOutput() {
    this->tamTrue_S = this->tam_S;
}

/*! This method takes the true values (tamTrue_S) and converts
 it over to an errored value.  It applies Gaussian noise, constant bias and scale factor to the truth. */
void Magnetometer::applySensorErrors() {
    //! - If any of the standard deviation vector elements is not positive, do not use noise error from RNG.
    bool anyNoiseComponentUninitialized = false;
    for (unsigned i = 0; i < this->senNoiseStd.size(); i++) {
        if ((this->senNoiseStd(i) <= 0.0)) { anyNoiseComponentUninitialized = true; }
    }
    if (anyNoiseComponentUninitialized) {
        this->tamSensed_S = this->tamTrue_S;
    } else {
        //! - Get current error from random number generator
        this->noiseModel.computeNextState();
        Eigen::Vector3d currentError = this->noiseModel.getCurrentState();
        //! - Sensed value with noise
        this->tamSensed_S = this->tamTrue_S + currentError;
    }
    //! - Sensed value with bias
    this->tamSensed_S = this->tamSensed_S + this->senBias;
    //! - Multiplying the sensed value with a scale factor
    this->tamSensed_S *= this->scaleFactor;
}

/*! This method applies saturation using the given bounds. */
void Magnetometer::applySaturation() {
    this->tamSensed_S = this->tamSensed_S.cwiseMax(this->minOutput).cwiseMin(this->maxOutput);
}

/*! This method writes the output messages. */
void Magnetometer::writeOutputMessages(uint64_t Clock) {
    TAMSensorMsgPayload localMessage;
    //! - Zero the output message
    localMessage = TAMSensorMsgPayload{};
    eigenVectorToCArray(this->tamSensed_S, localMessage.tam_S);
    //! - Write the outgoing message to the architecture
    this->tamDataOutMsg.write(localMessage, this->moduleID, Clock);
}

/*! This method is called at a specified rate by the architecture.  It makes the
 calls to compute the current magnetic field information and write the output message for
 the rest of the model.
 @param currentSimNanos The current simulation time from the architecture */
void Magnetometer::updateState(uint64_t currentSimNanos) {
    //! - Read the inputs
    this->readInputMessages();
    //! - Get magnetic field vector
    this->computeMagData();
    //! - Compute true output
    this->computeTrueOutput();
    //! - Apply any set errors
    this->applySensorErrors();
    //! - Apply saturation
    this->applySaturation();
    //! - Write output data
    this->writeOutputMessages(currentSimNanos);
}
