// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIMPLE_VOLT_ESTIMATOR_H
#define SIMPLE_VOLT_ESTIMATOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/VoltMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/gauss_markov.h>

#include <Eigen/Dense>
#include <vector>

//! 1x1 covariance / propagation / error type for the Gauss-Markov error model.
//! Named so SWIG can match it by name (see swig_eigen.i EIGEN_MAT_WRAP) and expose it to Python.
typedef Eigen::Matrix<double, 1, 1> VoltErrorMatrix;

/*! @brief simple voltage estimation module class */
class SimpleVoltEstimator : public SysModel {
public:
    SimpleVoltEstimator();
    ~SimpleVoltEstimator();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void computeErrors();
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

public:
    VoltErrorMatrix
        PMatrix;  //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    VoltErrorMatrix walkBounds;          //!< -- "3-sigma" errors to permit for states
    VoltErrorMatrix voltErrors;          //!< -- Current voltage errors applied to truth
    Message<VoltMsgPayload> voltOutMsg;  //!< voltage output msg
    VoltMsgPayload trueVoltState;        //!< -- voltage state without errors
    VoltMsgPayload estVoltState;         //!< -- voltage state including errors
    BSKLogger bskLogger;                 //!< -- BSK Logging

    ReadFunctor<VoltMsgPayload> voltInMsg;  //!< voltage input msg

private:
    VoltErrorMatrix AMatrix;  //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;   //!< -- Gauss-markov error states
};

#endif
