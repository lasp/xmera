// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module forceTorqueThrForceMapping
%{
    #include "forceTorqueThrForceMapping.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "forceTorqueThrForceMapping.h"
%include "forceTorqueThrForceMappingAlgorithm.h"

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
