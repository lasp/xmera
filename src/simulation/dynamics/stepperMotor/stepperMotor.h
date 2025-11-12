#ifndef _STEPPERMOTOR_
#define _STEPPERMOTOR_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
#include <stdint.h>

/*! @brief Stepper motor class. */
class StepperMotor : public SysModel {
   public:
    StepperMotor() = default;   //!< Constructor
    ~StepperMotor() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    double getThetaInit() const;                          //!< Getter method for the initial motor angle
    double getStepAngle() const;                          //!< Getter method for the motor step angle
    double getStepTime() const;                           //!< Getter method for the motor step time
    double getThetaDDotMax() const;                       //!< Getter method for the maximum motor angular acceleration
    void setThetaInit(const double thetaInit);            //!< Setter method for the initial motor angle
    void setStepAngle(const double stepAngle);            //!< Setter method for the motor step angle
    void setStepTime(const double stepTime);              //!< Setter method for the motor step time

    ReadFunctor<MotorStepCommandMsgPayload>
        motorStepCommandInMsg;                           //!< Input msg for the number of commanded motor step counts
    Message<StepperMotorMsgPayload> stepperMotorOutMsg;  //!< Output msg for the stepper motor state information

   private:
    void actuateMotor(double t);  //!< Method used to simulate the stepper motor actuation in time.
    void resetMotor();            //!< Method used to reset the motor states when the current request is complete and a
                                  //!< new request is received
    void updateStepParameters();  //!< Method used to update the step parameters after a step is completed
    bool isInStepFirstHalf(double t);  //!< Method used to determine if the motor is in the first half of a step
    void computeStepFirstHalf(
        double t);                      //!< Method used to compute the motor states during the first half of each step
    bool isInStepSecondHalf(double t);  //!< Method used to determine if the motor is in the second half of a step
    void computeStepSecondHalf(
        double t);  //!< Method used to compute the motor states during the second half of each step
    void computeStepComplete(double t);  //!< Method used to compute the motor states when a step is complete

    /* Step parameters */
    double stepAngle;    //!< [rad] Angle the stepper motor moves through for a single step (constant)
    double stepTime;     //!< [s] Time required for a single motor step (constant)
    int stepsCommanded;  //!< [steps] Number of commanded steps
    int stepCount;       //!< [steps] Current motor step count (number of steps taken)

    /* Motor angle parameters */
    double thetaInit;              //!< [rad] Initial motor angle
    double intermediateThetaInit;  //!< [rad] Initial motor angle at the start of each step
    double intermediateThetaRef;   //!< [rad] Reference motor angle at the end of each step
    double theta;                  //!< [rad] Current motor angle
    double thetaDot;               //!< [rad/s] Current motor angle rate
    double thetaDDot;              //!< [rad/s^2] Current motor angular acceleration
    double thetaDDotMax;           //!< [rad/s^2] Maximum angular acceleration of the stepper motor

    /* Temporal parameters */
    double tInit;                //!< [s] Simulation time at the beginning of each step
    double previousWrittenTime;  //!< [ns] Time the last input message was written
    double ts;                   //!< [s] The simulation time halfway through each step (switch time for ang accel)
    double tf;                   //!< [s] Simulation time when the current step will be completed

    /* Boolean parameters */
    bool actuationComplete;  //!< Boolean designating the motor has fully completed the commanded actuation
    bool stepComplete;       //!< Boolean designating the completion of a step
    bool newMsg;             //!< Boolean designating a new command message is written
    bool interruptMsg;       //!< Boolean designating the new command message is interrupting motor actuation

    /* Constant parameters */
    double a;  //!< Parabolic constant for the first half of a step
    double b;  //!< Parabolic constant for the second half of a step
};

#endif
