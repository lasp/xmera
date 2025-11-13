#ifndef STEPPERMOTORCONTROLLER_H
#define STEPPERMOTORCONTROLLER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include "stepperMotorControllerAlgorithm.h"

/*! @brief Stepper Motor Controller Class */
class StepperMotorController : public SysModel {
   public:
    StepperMotorController() = default;
    ~StepperMotorController() final = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setThetaInit(const double thetaInit);
    double getThetaInit() const;
    void setThetaMax(const double thetaMax);
    double getThetaMax() const;
    void setThetaMin(const double thetaMin);
    double getThetaMin() const;
    void setStepAngle(const double stepAngle);
    double getStepAngle() const;
    void setStepTime(const double stepTime);
    double getStepTime() const;

    ReadFunctor<HingedRigidBodyMsgPayload> motorRefAngleInMsg;   //!< Intput msg for the motor reference angle message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for the number of commanded motor steps

   private:
    StepperMotorControllerAlgorithm algorithm{};
};

#endif
