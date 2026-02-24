// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MOTOR_VOLTAGE_INTERFACE_H
#define MOTOR_VOLTAGE_INTERFACE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <vector>

#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/macroDefinitions.h>
#include <Eigen/Dense>

/*! @brief RW voltage interface class */
class MotorVoltageInterface : public SysModel {
   public:
    MotorVoltageInterface();
    ~MotorVoltageInterface();

    void computeMotorTorque();
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void setGains(Eigen::VectorXd gains);  //!< --     Takes in an array of gains to set for rws and sets them, leaving
                                           //!< blanks up to MAX_EFF_COUNT
    void setScaleFactors(Eigen::VectorXd scaleFactors);  //!< --     Takes in an array of scale factors to set for rws
                                                         //!< and sets them, leaving blanks up to MAX_EFF_COUNT
    void setBiases(Eigen::VectorXd biases);  //!< --     Takes in an array of biases to set for rws and sets them,
                                             //!< leaving blanks up to MAX_EFF_COUNT

   public:
    ReadFunctor<RwMotorVoltageMsgPayload>
        motorVoltageInMsg;                               //!< --     Message that contains motor voltage input states
    Message<RwMotorTorqueMsgPayload> motorTorqueOutMsg;  //!< --     Output Message for motor torques
    Eigen::VectorXd voltage2TorqueGain;                  //!< Nm/V   gain to convert voltage to motor torque
    Eigen::VectorXd scaleFactor;                         //!<        scale the output - like a constant gain error
    Eigen::VectorXd bias;                                //!< Nm     A bias to add to the torque output
    BSKLogger bskLogger;                                 //!< -- BSK Logging

   private:
    RwMotorTorqueMsgPayload outputTorqueBuffer;   //!< [Nm] copy of module output buffer
    uint64_t prevTime;                            //!< -- Previous simulation time observed
    RwMotorVoltageMsgPayload inputVoltageBuffer;  //!< [V] One-time allocation for time savings
};

#endif
