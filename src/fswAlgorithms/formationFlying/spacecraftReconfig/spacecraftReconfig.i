// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module spacecraftReconfig
%{
   #include "spacecraftReconfig.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "spacecraftReconfig.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/ReconfigBurnArrayInfoMsgPayload.h>
