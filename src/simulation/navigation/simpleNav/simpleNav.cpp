// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "simpleNav.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.h>

#include <cstring>
#include <iostream>

/*! This is the constructor for the simple nav model.  It sets default variable
    values and initializes the various parts of the model */
SimpleNav::SimpleNav() {
    this->crossTrans = false;
    this->crossAtt = false;
    this->prevTime = 0;
    this->estAttState = NavAttMsgPayload{};
    this->trueAttState = NavAttMsgPayload{};
    this->estTransState = NavTransMsgPayload{};
    this->trueTransState = NavTransMsgPayload{};
    this->accelDataState = AccDataMsgPayload{};
    this->spacecraftEphemerisState = EphemerisMsgPayload{};
    this->PMatrix.setZero();
    this->walkBounds.setZero();
    this->errorModel = GaussMarkov(18, this->RNGSeed);
}

/*! Destructor.  Nothing here. */
SimpleNav::~SimpleNav() {
    return;
}

/*! This method is used to reset the module. It
 initializes the various containers used in the model as well as creates the
 output message.  The error states are allocated as follows:
 Total states: 18
     - Position errors [0-2]
     - Velocity errors [3-5]
     - Attitude errors [6-8]
     - Body Rate errors [9-11]
     - Sun Point error [12-14]
     - Accumulated DV errors [15-17]
 @return void
 */
void SimpleNav::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->scStateInMsg.isLinked()) { bskLogger.bskLog(BSK_ERROR, "SimpleNav.scStateInMsg was not linked."); }

    //! - Initialize the propagation matrix to default values for use in update
    this->AMatrix.setIdentity();
    this->AMatrix(0, 3) = this->AMatrix(1, 4) = this->AMatrix(2, 5) = this->crossTrans ? 1.0 : 0.0;
    this->AMatrix(6, 9) = this->AMatrix(7, 10) = this->AMatrix(8, 11) = this->crossAtt ? 1.0 : 0.0;

    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
    vSetZero(this->gyroErrors, 3 * MAX_ACC_BUF_PKT);
    vSetZero(this->accelErrors, 3 * MAX_ACC_BUF_PKT);
}

/*! This method reads the input messages associated with the vehicle state and
 the sun state
 */
void SimpleNav::readInputMessages() {
    this->inertialState = this->scStateInMsg();

    this->sunState = SpicePlanetStateMsgPayload{};
    if (this->sunStateInMsg.isLinked()) { this->sunState = this->sunStateInMsg(); }
}

/*! This method writes the aggregate nav information into the output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void SimpleNav::writeOutputMessages(uint64_t Clock) {
    /* time tage the output message */
    this->estAttState.timeTag = (double) Clock * NANO2SEC;
    this->estTransState.timeTag = (double) Clock * NANO2SEC;
    this->spacecraftEphemerisState.timeTag = (double) Clock * NANO2SEC;

    this->attOutMsg.write(&this->estAttState, this->moduleID, Clock);
    this->transOutMsg.write(&this->estTransState, this->moduleID, Clock);
    this->scEphemOutMsg.write(&this->spacecraftEphemerisState, this->moduleID, Clock);
    this->accelDataOutMsg.write(&this->accelDataState, this->moduleID, Clock);
}

void SimpleNav::applyErrors() {
    //! - Add errors to the simple cases (everything except sun-pointing)
    v3Add(this->trueTransState.r_BN_N, &(this->navErrors.data()[0]), this->estTransState.r_BN_N);
    v3Add(this->trueTransState.v_BN_N, &(this->navErrors.data()[3]), this->estTransState.v_BN_N);
    addMRP(this->trueAttState.sigma_BN, &(this->navErrors.data()[6]), this->estAttState.sigma_BN);
    v3Add(this->trueAttState.omega_BN_B, &(this->navErrors.data()[9]), this->estAttState.omega_BN_B);
    v3Add(
        this->spacecraftEphemerisState.r_BdyZero_N,
        &(this->navErrors.data()[0]),
        this->spacecraftEphemerisState.r_BdyZero_N
    );
    v3Add(
        this->spacecraftEphemerisState.v_BdyZero_N,
        &(this->navErrors.data()[3]),
        this->spacecraftEphemerisState.v_BdyZero_N
    );
    addMRP(
        this->spacecraftEphemerisState.sigma_BN,
        &(this->navErrors.data()[6]),
        this->spacecraftEphemerisState.sigma_BN
    );
    v3Add(
        this->spacecraftEphemerisState.omega_BN_B,
        &(this->navErrors.data()[9]),
        this->spacecraftEphemerisState.omega_BN_B
    );
    v3Add(this->trueTransState.vehAccumDV, &(this->navErrors.data()[15]), this->estTransState.vehAccumDV);

    //! - Apply accelerometer errors to truth data
    for (int index = 0; index < this->numberOfGyroBuffers; ++index) {
        AccPktDataMsgPayload accelPacketPayload = this->accelDataState.accPkts[index];
        v3Add(accelPacketPayload.gyro_B, &this->gyroErrors[3 * index], accelPacketPayload.gyro_B);
        v3Add(accelPacketPayload.accel_B, &this->accelErrors[3 * index], accelPacketPayload.accel_B);
        this->accelDataState.accPkts[index] = accelPacketPayload;
    }

    //! - Add errors to  sun-pointing
    if (this->sunStateInMsg.isLinked()) {
        double dcm_OT[3][3]; /* dcm, body T to body O */
        MRP2C(&(this->navErrors.data()[12]), dcm_OT);
        m33MultV3(dcm_OT, this->trueAttState.vehSunPntBdy, this->estAttState.vehSunPntBdy);
        v3Normalize(this->estAttState.vehSunPntBdy, this->estAttState.vehSunPntBdy);
    } else {
        v3SetZero(this->estAttState.vehSunPntBdy);
    }
}

/*! This method uses the input messages as well as the calculated model errors to
 compute what the output navigation state should be.
    @return void
    @param Clock The clock time associated with the model's update call
*/
void SimpleNav::computeTrueOutput(uint64_t Clock) {
    //! - Set output state to truth data
    v3Copy(this->inertialState.r_BN_N, this->trueTransState.r_BN_N);
    v3Copy(this->inertialState.v_BN_N, this->trueTransState.v_BN_N);
    v3Copy(this->inertialState.sigma_BN, this->trueAttState.sigma_BN);
    v3Copy(this->inertialState.omega_BN_B, this->trueAttState.omega_BN_B);
    v3Copy(this->inertialState.TotalAccumDVBdy, this->trueTransState.vehAccumDV);

    //! - Set accelerometer state to truth data
    for (int index = 0; index < this->numberOfGyroBuffers; ++index) {
        AccPktDataMsgPayload accelPacketPayload;
        uint64_t timeOffset = index * SEC2NANO / this->gyroFrequencyPerSecond;
        accelPacketPayload.measTime = Clock + timeOffset;
        v3Copy(this->inertialState.omega_BN_B, accelPacketPayload.gyro_B);
        v3Copy(this->inertialState.omegaDot_BN_B, accelPacketPayload.accel_B);
        this->accelDataState.accPkts[index] = accelPacketPayload;
    }

    //! - Set ephemeris state to truth data
    v3Copy(this->inertialState.r_BN_N, this->spacecraftEphemerisState.r_BdyZero_N);
    v3Copy(this->inertialState.v_BN_N, this->spacecraftEphemerisState.v_BdyZero_N);
    v3Copy(this->inertialState.sigma_BN, this->spacecraftEphemerisState.sigma_BN);
    v3Copy(this->inertialState.omega_BN_B, this->spacecraftEphemerisState.omega_BN_B);

    //! - For the sun pointing output, compute the spacecraft to sun vector, normalize, and trans 2 body.
    if (this->sunStateInMsg.isLinked()) {
        double sc2SunInrtl[3];
        double dcm_BN[3][3]; /* dcm, inertial to body */
        v3Subtract(this->sunState.PositionVector, this->inertialState.r_BN_N, sc2SunInrtl);
        v3Normalize(sc2SunInrtl, sc2SunInrtl);
        MRP2C(this->inertialState.sigma_BN, dcm_BN);
        m33MultV3(dcm_BN, sc2SunInrtl, this->trueAttState.vehSunPntBdy);
        v3Normalize(this->trueAttState.vehSunPntBdy, this->trueAttState.vehSunPntBdy);
    } else {
        v3SetZero(this->trueAttState.vehSunPntBdy);
    }
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.
 @return void
 @param currentSimNanos The clock time associated with the model call
 */
void SimpleNav::computeErrors(uint64_t currentSimNanos) {
    double timeStep;
    SimpleNavCovar localProp = this->AMatrix;
    //! - Compute timestep since the last call
    timeStep = (currentSimNanos - this->prevTime) * 1.0E-9;

    localProp(0, 3) *= timeStep;   // postion/velocity cross correlation terms
    localProp(1, 4) *= timeStep;   // postion/velocity cross correlation terms
    localProp(2, 5) *= timeStep;   // postion/velocity cross correlation terms
    localProp(6, 9) *= timeStep;   // attitude/attitude rate cross correlation terms
    localProp(7, 10) *= timeStep;  // attitude/attitude rate cross correlation terms
    localProp(8, 11) *= timeStep;  // attitude/attitude rate cross correlation terms

    //! - Set the GaussMarkov propagation matrix and compute errors
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();

    //! - Compute accelerometer errors
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<double> gyroErrorDistribution(this->gyroBias, this->gyroStandardDeviation);
    std::normal_distribution<double> accelErrorDistribution(this->accelBias, this->accelStandardDeviation);
    for (int index = 0; index < 3 * this->numberOfGyroBuffers; ++index) {
        this->gyroErrors[index] = gyroErrorDistribution(generator);
        this->accelErrors[index] = accelErrorDistribution(generator);
    }
}

/*! This method calls all of the run-time operations for the simple nav model.
    @return void
    @param currentSimNanos The clock time associated with the model call
*/
void SimpleNav::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
    this->computeTrueOutput(currentSimNanos);
    this->computeErrors(currentSimNanos);
    this->applyErrors();
    this->writeOutputMessages(currentSimNanos);
    this->prevTime = currentSimNanos;
}
