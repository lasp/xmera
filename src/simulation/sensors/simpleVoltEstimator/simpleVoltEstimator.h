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
    Eigen::MatrixXd
        PMatrix;  //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::VectorXd walkBounds;          //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd voltErrors;          //!< -- Current voltage errors applied to truth
    Message<VoltMsgPayload> voltOutMsg;  //!< voltage output msg
    VoltMsgPayload trueVoltState;        //!< -- voltage state without errors
    VoltMsgPayload estVoltState;         //!< -- voltage state including errors
    BSKLogger bskLogger;                 //!< -- BSK Logging

    ReadFunctor<VoltMsgPayload> voltInMsg;  //!< voltage input msg

   private:
    Eigen::MatrixXd AMatrix;  //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;   //!< -- Gauss-markov error states
};

#endif
