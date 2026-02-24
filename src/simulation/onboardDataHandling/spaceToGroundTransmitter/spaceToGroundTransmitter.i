// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module spaceToGroundTransmitter
%{
#include "spaceToGroundTransmitter.h"
%}

%include <architecture/_GeneralModuleFiles/swig_common_model.i>
%include <carrays.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>
%include "spaceToGroundTransmitter.h"

%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>

%include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
%include <architecture/msgPayloadDef/AccessMsgPayload.h>
