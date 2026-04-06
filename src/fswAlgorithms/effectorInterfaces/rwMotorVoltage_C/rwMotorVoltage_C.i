// SPDX-License-Identifier: ISC
// Copyright (c) 2017, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module rwMotorVoltage_C
%{
   #include "rwMotorVoltage_C.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rwMotorVoltage_C.h"

%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>

%include <fswAlgorithms/fswUtilities/fswDefinitions.h>
%include <architecture/utilities/macroDefinitions.h>
