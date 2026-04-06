// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module scanningInstrumentController
%{
    #include "scanningInstrumentController.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "scanningInstrumentController.h"

%include <architecture/msgPayloadDef/AccessMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
