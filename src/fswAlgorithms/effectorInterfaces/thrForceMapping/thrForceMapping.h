// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_THRUSTER_FORCE_MAPPING_H
#define XMERA_THRUSTER_FORCE_MAPPING_H

#include "thrForceMappingAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <stdint.h>

#include <Eigen/Dense>

/*!@brief Thruster force mapping class. */
class ThrForceMapping : public SysModel {
public:
    ThrForceMapping() = default;                   //!< Constructor
    ~ThrForceMapping() = default;                  //!< Destructor
    void reset(uint64_t callTime) override;        //!< Reset method
    void updateState(uint64_t callTime) override;  //!< Update method

    Eigen::Matrix3d getControlAxesB() const;            //!< Getter method for thruster control axes
    void setControlAxesB(Eigen::Matrix3d const &axes);  //!< Setter method for thruster control axes
    Vector36d getThrForceMag() const;                   //!< Getter method for the thruster force magnitude
    void setThrForceMag(Vector36d const &forceMag);     //!< Setter method for the thruster force magnitude
    ThrForceSign getThrForceSign() const;               //!< Getter method for the thruster force sign
    void setThrForceSign(ThrForceSign sign);            //!< Setter method for the thruster force sign
    double getAngErrThresh() const;                     //!< Getter method for the angular error threshold
    void setAngErrThresh(double thresh);                //!< Setter method for the angular error threshold
    double getEpsilon() const;                          //!< Getter method for the epsilon value
    void setEpsilon(double eps);                        //!< Setter method for the epsilon value
    bool getUse2ndLoop() const;  //!< Getter method for whether the second least squares fitting loop should be used
    void setUse2ndLoop(
        bool loopFlag
    );  //!< Getter method for whether the second least squares fitting loop should be used

    Message<THRArrayCmdForceMsgPayload> thrForceCmdOutMsg;  //!< The name of the output thruster force message
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;    //!< The name of the vehicle control (Lr) Input message
    ReadFunctor<THRArrayConfigMsgPayload> thrConfigInMsg;   //!< The name of the thruster cluster Input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;    //!< The name of the Input message


private:
    ThrForceMappingAlgorithm algorithm;  //!< Algorithm for thrForceMapping logic (BSK-agnostic)
};

#endif
