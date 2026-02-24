// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SINGLEAXISPROFILER_
#define _SINGLEAXISPROFILER_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
#include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Single Axis Profiler Class */
class SingleAxisProfiler : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    void setRotHat_M(const Eigen::Vector3d& rotHat_M);    //!< Setter for the spinning body rotation axis
    const Eigen::Vector3d& getRotHat_M() const;           //!< Getter for the spinning body rotation axis

    ReadFunctor<StepperMotorMsgPayload> stepperMotorInMsg;  //!< Input msg for the stepper motor state information
    Message<PrescribedRotationMsgPayload>
        prescribedRotationOutMsg;  //!< Output msg for the hub-relative prescribed rotational states

    BSKLogger* bskLogger;  //!< BSK Logging

   private:
    Eigen::Vector3d computeSigma_FM(double theta);  //!< Method for computing the current spinning body MRP attitude
                                                    //!< relative to the mount frame: sigma_FM
    Eigen::Vector3d rotHat_M;                       //!< Spinning body rotation axis expressed in M frame components
};

#endif
