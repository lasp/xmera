// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef HINGEDBODYLINEARPROFILER_H
#define HINGEDBODYLINEARPROFILER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Linear deployment profiler for single hinged rigid body.
 */
class HingedBodyLinearProfiler : public SysModel {
   public:
    HingedBodyLinearProfiler();
    ~HingedBodyLinearProfiler();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

   public:
    uint64_t startTime;  //!< [ns] time to begin deployment
    uint64_t endTime;    //!< [ns] time to end deployment
    double startTheta;   //!< [rad] starting hinged rigid body theta position
    double endTheta;     //!<  [rad] ending hinged rigid body theta position

    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyReferenceOutMsg;  //!< -- output message for reference hinged rigid body state (theta, theta dot)

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    double deploymentSlope;  //!<  [rad/s] slope of deployment
};

#endif
