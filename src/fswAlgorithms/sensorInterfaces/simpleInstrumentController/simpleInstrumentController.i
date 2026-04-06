// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleInstrumentController
%{
#include "simpleInstrumentController.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "simpleInstrumentController.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/AccessMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
