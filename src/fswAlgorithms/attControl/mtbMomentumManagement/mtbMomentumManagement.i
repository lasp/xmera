// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module mtbMomentumManagement
%{
   #include "mtbMomentumManagement.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "mtbMomentumManagement.h"

// sample Module support file to be included in this sub-module
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
