// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include "stepperMotorControllerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>

/*! @brief Stepper Motor Controller Class */
class StepperMotorController final : public SysModel {
public:
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setThetaInit(double const thetaInit);
    double getThetaInit() const;
    void setThetaMax(double const thetaMax);
    double getThetaMax() const;
    void setThetaMin(double const thetaMin);
    double getThetaMin() const;
    void setStepAngle(double const stepAngle);
    double getStepAngle() const;
    void setStepTime(double const stepTime);
    double getStepTime() const;

    ReadFunctor<HingedRigidBodyMsgPayload> motorRefAngleInMsg;   //!< Intput msg for the motor reference angle message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for the number of commanded motor steps

private:
    StepperMotorControllerAlgorithm algorithm{};
};

#endif
