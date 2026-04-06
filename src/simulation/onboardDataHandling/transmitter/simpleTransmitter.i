// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleTransmitter
%{
    #include "simpleTransmitter.h"
%}

%include <architecture/_GeneralModuleFiles/swig_common_model.i>

%include <std_string.i>
%include <std_vector.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <stdint.i>

%include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>
%include "simpleTransmitter.h"

%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>

%include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
