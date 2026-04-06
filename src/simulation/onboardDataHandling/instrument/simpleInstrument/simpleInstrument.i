// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleInstrument
%{
#include "simpleInstrument.h"
%}

%include <std_string.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>
%include "simpleInstrument.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>

%include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
