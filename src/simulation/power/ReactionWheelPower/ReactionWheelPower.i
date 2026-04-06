// SPDX-License-Identifier: ISC
// Copyright (c) 2020, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module ReactionWheelPower
%{
    #include "ReactionWheelPower.h"
%}

%include <stdint.i>
%include <std_string.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>
%include "ReactionWheelPower.h"

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>

%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
