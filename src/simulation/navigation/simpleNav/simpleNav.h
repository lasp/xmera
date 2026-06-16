// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIMPLE_NAV_H
#define SIMPLE_NAV_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccDataMsgPayload.h>
#include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/gauss_markov.h>

#include <Eigen/Dense>
#include <random>
#include <vector>

//! 18x18 process-noise / propagation matrix type for the Gauss-Markov error model.
//! Named so SWIG can match it by name (see swig_eigen.i EIGEN_MAT_WRAP) and expose it to Python.
typedef Eigen::Matrix<double, 18, 18> SimpleNavCovar;
//! 18-element error / walk-bounds vector type for the Gauss-Markov error model.
typedef Eigen::Matrix<double, 18, 1> SimpleNavErrorVector;

/*! @brief simple navigation module class */
class SimpleNav : public SysModel {
public:
    SimpleNav();
    ~SimpleNav();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void computeTrueOutput(uint64_t Clock);
    void computeErrors(uint64_t currentSimNanos);
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

public:
    double gyroStandardDeviation = 1E-5;      //!< Standard deviation for each rate component
    double accelStandardDeviation = 1E-8;     //!< Standard deviation for each acceleration component
    double gyroBias = 0;                      //!<  Bias for each rate component
    double accelBias = 0;                     //!<  Bias for each acceleration component
    double gyroErrors[3 * MAX_ACC_BUF_PKT];   //!<  Errors to apply to each gyro measurement
    double accelErrors[3 * MAX_ACC_BUF_PKT];  //!<  Errors to apply to each accelerometer measurement
    int numberOfGyroBuffers = 100;            //!< Number of gyro measurements per timestep
    int gyroFrequencyPerSecond = 500;         //!< Number of gyro measurements per second
    SimpleNavCovar
        PMatrix;  //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    SimpleNavErrorVector walkBounds;               //!< -- "3-sigma" errors to permit for states
    SimpleNavErrorVector navErrors;                //!< -- Current navigation errors applied to truth
    Message<NavAttMsgPayload> attOutMsg;           //!< attitude navigation output msg
    Message<NavTransMsgPayload> transOutMsg;       //!< translation navigation output msg
    Message<EphemerisMsgPayload> scEphemOutMsg;    //!< translation navigation output msg
    Message<AccDataMsgPayload> accelDataOutMsg;    //!< accelerometer and gyro data output msg
    bool crossTrans;                               //!< -- Have position error depend on velocity
    bool crossAtt;                                 //!< -- Have attitude depend on attitude rate
    NavAttMsgPayload trueAttState;                 //!< -- attitude nav state without errors
    NavAttMsgPayload estAttState;                  //!< -- attitude nav state including errors
    NavTransMsgPayload trueTransState;             //!< -- translation nav state without errors
    NavTransMsgPayload estTransState;              //!< -- translation nav state including errors
    EphemerisMsgPayload spacecraftEphemerisState;  //!< -- full spacecraft ephemeris state with errors
    AccDataMsgPayload accelDataState;              //!< accelerometer and gyro data payload
    SCStatesMsgPayload inertialState;              //!< -- input inertial state from Star Tracker
    SpicePlanetStateMsgPayload sunState;           //!< -- input Sun state
    BSKLogger bskLogger;                           //!< -- BSK Logging

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;           //!< spacecraft state input msg
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;  //!< (optional) sun state input input msg

private:
    SimpleNavCovar AMatrix;  //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;  //!< -- Gauss-markov error states
    uint64_t prevTime;       //!< -- Previous simulation time observed
};

#endif
