// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module hingedRigidBodyMotorSensor
%{
    #include "hingedRigidBodyMotorSensor.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "hingedRigidBodyMotorSensor.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
