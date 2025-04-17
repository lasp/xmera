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

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <Eigen/Dense>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/STSensorMsgPayload.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/gauss_markov.h"

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

   public:
    uint64_t sensorTimeTag = 0;                    //!< [ns] Current time tag for sensor out
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;  //!< Sc input state message
    Message<STSensorMsgPayload> sensorOutMsg;      //!< Sensor output state message

    Eigen::Matrix3d
        PMatrix;  //!< Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::Vector3d walkBounds{0.0, 0.0, 0.0};  //!< "3-sigma" errors to permit for states
    Eigen::Vector3d navErrors{0.0, 0.0, 0.0};   //!< Current navigation errors applied to truth

    Eigen::Matrix3d dcm_CB;                    //!< Transformation matrix from body to case
    STSensorMsgPayload trueValues;             //!< Total measurement without perturbations
    STSensorMsgPayload sensedValues;           //!< Total measurement including perturbations
    Eigen::Vector3d mrpErrors{0.0, 0.0, 0.0};  //!< Errors to be applied to the input MRP set indicating whether
    SCStatesMsgPayload scState;                //!< Module variable where the input State Data message is stored
    BSKLogger bskLogger;                       //!< BSK Logging

   private:
    Eigen::Matrix3d AMatrix;  //!< AMatrix that we use for error propagation
    GaussMarkov errorModel;   //!< Gauss-markov error states

    uint64_t previousSimTime = 0;  //!< [ns] Previous sim time
    Eigen::Vector4d betaPrevious_CN{1.0,
                                    0.0,
                                    0.0,
                                    0.0};  //!< Previous sensed quaternion from inertial to platform case frame
};

#endif
