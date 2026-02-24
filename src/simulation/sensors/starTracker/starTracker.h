// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <Eigen/Dense>
#include <vector>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/STSensorMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/gauss_markov.h>
#include <architecture/utilities/macroDefinitions.h>

/*! @brief star tracker class */
class StarTracker : public SysModel {
   public:
    StarTracker();
    ~StarTracker();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t CurrentClock);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void computeSensorErrors();
    void applySensorErrors();
    void computeTrueOutput();
    void computeQuaternion(Eigen::Vector3d* sigma, STSensorMsgPayload* sensorValue);
    void computeAngularVelocity(uint64_t currentSimNanos);
    void setDcmCB(const Eigen::Matrix3d& dcm_CB);
    void setPMatrix(const Eigen::Matrix3d& PMatrix);
    void setWalkBounds(const Eigen::Vector3d& walkBounds);
    const Eigen::Matrix3d& getDcmCB() const;
    const Eigen::Matrix3d& getPMatrix() const;
    const Eigen::Vector3d& getWalkBounds() const;

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;  //!< Sc input state message
    Message<STSensorMsgPayload> sensorOutMsg;      //!< Sensor output state message

    BSKLogger bskLogger;  //!< BSK Logging

   private:
    uint64_t sensorTimeTag = 0;  //!< [ns] Current time tag for sensor out
    Eigen::Matrix3d
        PMatrix;  //!< Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::Vector3d walkBounds{0.0, 0.0, 0.0};  //!< "3-sigma" errors to permit for states
    Eigen::Vector3d navErrors{0.0, 0.0, 0.0};   //!< Current navigation errors applied to truth
    Eigen::Matrix3d dcm_CB;                     //!< Transformation matrix from body to case
    STSensorMsgPayload trueValues;              //!< Total measurement without perturbations
    STSensorMsgPayload sensedValues;            //!< Total measurement including perturbations
    Eigen::Vector3d mrpErrors{0.0, 0.0, 0.0};   //!< Errors to be applied to the input MRP set indicating whether
    SCStatesMsgPayload scState;                 //!< Module variable where the input State Data message is stored
    Eigen::Matrix3d AMatrix;                    //!< AMatrix that we use for error propagation
    GaussMarkov errorModel;                     //!< Gauss-markov error states
    uint64_t previousSimTime = 0;               //!< [ns] Previous sim time
    Eigen::Vector4d betaPrevious_CN{1.0,
                                    0.0,
                                    0.0,
                                    0.0};  //!< Previous sensed quaternion from inertial to platform case frame
};

#endif
