// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef REACTIONWHEELSTATEEFFECTOR_H
#define REACTIONWHEELSTATEEFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWCmdMsgPayload.h>
#include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
#include <architecture/msgPayloadDef/RWConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>

#include <Eigen/Dense>

/*! @brief reaction wheel state effector class */
class ReactionWheelStateEffector
    : public SysModel
    , public StateEffector {
public:
    ReactionWheelStateEffector() = default;
    ~ReactionWheelStateEffector();
    void registerStates(DynParamManager &states) override;
    void linkInStates(DynParamManager &states) override;
    void writeOutputStateMessages(uint64_t integTimeNanos) override;
    void computeDerivatives(
        double integTime,
        Eigen::Vector3d rDDotNonGrav_BN_N,
        Eigen::Vector3d omegaDot_BN_B,
        Eigen::Vector3d sigma_BN
    ) override;
    void updateEffectorMassProps(
        double integTime
    ) override;  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(
        double integTime,
        BackSubMatrices &backSubContr,
        Eigen::Vector3d sigma_BN,
        Eigen::Vector3d omega_BN_B
    ) override;  //!< -- Back-sub contributions
    void updateEnergyMomContributions(
        double integTime,
        Eigen::Vector3d &rotAngMomPntCContr_B,
        double &rotEnergyContr,
        Eigen::Vector3d omega_BN_B
    ) override;  //!< -- Energy and momentum calculations
    void reset(uint64_t currentSimNanos) override;
    void addReactionWheel(RWConfigMsgPayload* NewRW);
    void updateState(uint64_t currentSimNanos) override;
    void WriteOutputMessages(uint64_t CurrentClock) const;
    void ReadInputs();
    void ConfigureRWRequests(double CurrentTime);

public:
    std::vector<RWConfigMsgPayload*> ReactionWheelData;  //!< -- RW information

    ReadFunctor<RwMotorTorqueMsgPayload> rwMotorCmdInMsg;    //!< -- RW motor torque array cmd input message
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;                //!< -- RW speed array output message
    std::vector<Message<RWConfigLogMsgPayload>*> rwOutMsgs;  //!< -- vector of RW log output messages

    std::vector<RWCmdMsgPayload> NewRWCmds;                              //!< -- Incoming attitude commands
    RWSpeedMsgPayload rwSpeedMsgBuffer = {};                             //!< (-) Output data from the reaction wheels
    std::string nameOfReactionWheelOmegasState = "reactionWheelOmegas";  //!< wheel angular rate state name
    std::string nameOfReactionWheelThetasState = "reactionWheelThetas";  //!< wheel angle state name
    size_t numRW = 0;                                                    //!< number of reaction wheels
    size_t numRWJitter = 0;                                              //!< number of RW with jitter

private:
    RwMotorTorqueMsgPayload incomingCmdBuffer = {};  //!< -- One-time allocation for savings
    uint64_t prevCommandTime = UINT64_MAX;           //!< -- Time for previous valid thruster firing

    StateData* hubSigma = nullptr;
    StateData* hubOmega = nullptr;
    StateData* OmegasState = nullptr;
    StateData* thetasState = nullptr;
};

#endif /* REACTIONWHEELSTATEEFFECTOR_H */
